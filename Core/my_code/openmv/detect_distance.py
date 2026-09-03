# OpenMV -> STM32  USART2 visual transmission test.
# Frame: V,color,cx,cy,distance_cm\n
# color: 0=none, 1=red, 2=yellow, 3=black unload area.
# STM32 command: M,0\n=block mode, M,1\n=black unload area mode.
# Wall frame: W,state,fill_pct\n; state: 0=clear, 1=too close.
# fill_pct is the blue share of the wall ROI in percent, for tuning only.
# Arrival frame: A,state,fill_pct\n; fill_pct uses actual black pixels.

import sensor
import time
from pyb import LED, UART

# UART3 pins on OpenMV H7 Plus: P4=TX, P5=RX.
# Connect P4 to STM32 PA3 (USART2_RX), P5 to STM32 PA2 (USART2_TX),
# and connect both GND pins.
uart = UART(3, 115200, bits=8, parity=None, stop=1, timeout_char=20)

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
RED_TYPE = 1
YELLOW_TYPE = 2
BLACK_TYPE = 3

# -1 = wait for STM32 command, 0 = red/yellow blocks, 1 = black unload area.
# Starting idle ensures target locking begins only after an explicit M,0.
detection_mode = 0

# Latched wall flag. Kept across frames so the enter/exit thresholds
# can act as hysteresis instead of both being compared every frame.
wall_state = 0

# Red/yellow target lock. A locked target is matched by color, position, and
# apparent size instead of selecting the largest blob again every frame.
locked_target = None
LOCK_LOST_PAUSE_MS = 1300
lock_pause_started_ms = None
LOCK_MAX_MOVE_PX = 100
LOCK_MIN_SIZE_RATIO = 0.35
LOCK_MAX_SIZE_RATIO = 3.0

def read_stm32_mode():
    """Apply the latest M,0 / M,1 command sent by the STM32."""
    global detection_mode, locked_target, lock_pause_started_ms
    while uart.any():
        line = uart.readline()
        if not line:
            continue
        try:
            text = line.decode().strip()
        except Exception:
            continue
        if text == "M,0":
            detection_mode = 0
        elif text == "M,1":
            detection_mode = 1
            locked_target = None
            lock_pause_started_ms = None


def send_target(color, cx, cy, distance_cm):
    uart.write("V,%d,%d,%d,%d\n" % (
        color,
        cx,
        cy,
        distance_cm,
    ))


def send_wall(wall_state, fill_pct):
    uart.write("W,%d,%d\n" % (wall_state, fill_pct))


def send_arrival(arrived, fill_pct):
    uart.write("A,%d,%d\n" % (arrived, fill_pct))


def largest_blob(candidates):
    """Return (blob, color config) having the largest pixel count."""
    target = None
    for candidate in candidates:
        if target is None or candidate[0].pixels > target[0].pixels:
            target = candidate
    return target


def box_overlap_ratio(inner_blob, outer_blob):
    """Return how much of inner_blob's axis-aligned box is covered by outer_blob."""
    left = max(inner_blob.x, outer_blob.x)
    top = max(inner_blob.y, outer_blob.y)
    right = min(inner_blob.x + inner_blob.w, outer_blob.x + outer_blob.w)
    bottom = min(inner_blob.y + inner_blob.h, outer_blob.y + outer_blob.h)

    if right <= left or bottom <= top:
        return 0.0

    return ((right - left) * (bottom - top)) / (inner_blob.w * inner_blob.h)


def diagonal_intersection(corners, fallback_x, fallback_y):
    """Return the intersection of diagonals 0-2 and 1-3."""
    x1, y1 = corners[0]
    x2, y2 = corners[2]
    x3, y3 = corners[1]
    x4, y4 = corners[3]

    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if denominator == 0:
        return fallback_x, fallback_y

    first_cross = x1 * y2 - y1 * x2
    second_cross = x3 * y4 - y3 * x4
    center_x = int(
        (first_cross * (x3 - x4) - (x1 - x2) * second_cross)
        / denominator
    )
    center_y = int(
        (first_cross * (y3 - y4) - (y1 - y2) * second_cross)
        / denominator
    )
    return center_x, center_y+10


def matching_locked_blob(candidates, lock):
    """Return the candidate most similar to the previously locked target."""
    best = None
    best_score = None
    old_type, old_cx, old_cy, old_w, old_h = lock

    for candidate in candidates:
        blob = candidate[0]
        if candidate[1] != old_type:
            continue

        dx = blob.cx - old_cx
        dy = blob.cy - old_cy
        if abs(dx) > LOCK_MAX_MOVE_PX or abs(dy) > LOCK_MAX_MOVE_PX:
            continue

        width_ratio = blob.w / old_w
        height_ratio = blob.h / old_h
        if width_ratio < LOCK_MIN_SIZE_RATIO or width_ratio > LOCK_MAX_SIZE_RATIO:
            continue
        if height_ratio < LOCK_MIN_SIZE_RATIO or height_ratio > LOCK_MAX_SIZE_RATIO:
            continue

        # Position is the strongest identity cue; size changes as the car moves.
        size_error = abs(blob.w - old_w) + abs(blob.h - old_h)
        score = (dx * dx) + (dy * dy) + (size_error * size_error)
        if best is None or score < best_score:
            best = candidate
            best_score = score

    return best
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

green_led = LED(2)
blue_led = LED(3)
for _ in range(3):
    green_led.on()
    time.sleep_ms(200)
    green_led.off()
    time.sleep_ms(200)

# ROI: (x, y, w, h)
roi = (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)

# LAB threshold: (L_min, L_max, a_min, a_max, b_min, b_max)
# Format: (object_type, name, LAB threshold, drawing color, real object width in millimeters)
# Replace each width with the measured width of that color's object.
color_configs = (
    (RED_TYPE, "RED", (20, 75, 20, 127, -20, 70), (255, 0, 0), 25.0),
    (YELLOW_TYPE, "YELLOW", (43, 100, -29, 10, 31, 95), (255, 255, 0), 30.0),
)

# Black unload area. Threshold must be re-tuned on the real field.
BLACK_CONFIG = (BLACK_TYPE, "BLACK", (0, 35, -15, 18, -21, 10), (0, 0, 255), 400.0)
# The area is a large floor region, so it needs a bigger blob and merging.
BLACK_PIXELS_THRESHOLD = 300
BLACK_ARRIVAL_FILL_PCT = 15
# In red/yellow mode, ignore an object covered by this proportion of the
# largest black unload-area bounding box.
BLACK_OVERLAP_REJECT_RATIO = 0.90
# Reported distance per pixel of gap below the area's near edge.
# Increase it if the car stops too early, decrease it if it overshoots.
BLACK_NEAR_EDGE_CM_PER_PX = 0.3

# Always-enabled blue wall collision region. Its distance is estimated from
# the blue wall's apparent width using a measured real width.
WALL_ROI = (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT - 0)
WALL_ROI_AREA_PX = WALL_ROI[2] * WALL_ROI[3]
BLUE_WALL_THRESHOLD = (0, 68, -30, 20, -59, -13)
BLUE_WALL_COLOR = (0, 120, 255)
WALL_MIN_WIDTH_PX = 15
WALL_MIN_HEIGHT_PX = 10
WALL_PIXELS_THRESHOLD = 60

# The wall is judged by how much of the ROI is blue, not by an estimated
# distance: a wall wider than the frame saturates any width-based distance,
# but its pixel share keeps growing all the way in.
# Two thresholds give hysteresis, so the flag does not chatter at the edge.
# Calibrate by reading the percentage reported to the STM32.
WALL_FILL_ENTER_PCT = 95
WALL_FILL_EXIT_PCT = 75

# Smaller values make detection more sensitive, but may also detect noise.
# QVGA blob area is a quarter of the VGA area for the same object.
PIXELS_THRESHOLD = 20
AREA_THRESHOLD = 20

# Shape filters for cube/cylinder side views
# Reject thin wires, tiny regions, and sparse color noise.
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 2.0
MIN_BLOB_WIDTH = 4
MIN_BLOB_HEIGHT = 4

# QVGA focal-length calibration used for red/yellow block distance.
FOCAL_LENGTH_PX = 185

clock = time.clock()

while True:
    clock.tick()
    read_stm32_mode()
    img = sensor.snapshot()
    candidates = []

    lock_pause_active = False
    if lock_pause_started_ms is not None:
        elapsed_ms = time.ticks_diff(time.ticks_ms(), lock_pause_started_ms)
        if elapsed_ms < LOCK_LOST_PAUSE_MS:
            lock_pause_active = True
        else:
            lock_pause_started_ms = None

    # This collision check always runs, regardless of the STM32-selected mode.
    # Find the largest blue blob only and use its bounding box area ratio.
    wall_blob = None
    for blob in img.find_blobs(
        [BLUE_WALL_THRESHOLD],
        roi=WALL_ROI,
        pixels_threshold=WALL_PIXELS_THRESHOLD,
        area_threshold=WALL_PIXELS_THRESHOLD,
        merge=True,
    ):
        if blob.w < WALL_MIN_WIDTH_PX or blob.h < WALL_MIN_HEIGHT_PX:
            continue
        if wall_blob is None or blob.pixels > wall_blob.pixels:
            wall_blob = blob

    # Clamped because the STM32 rejects the frame as malformed above 100.
    if wall_blob is None:
        wall_fill_pct = 0
    else:
        wall_fill_pct = min(100, int(100 * wall_blob.w * wall_blob.h / WALL_ROI_AREA_PX))
    if wall_fill_pct >= WALL_FILL_ENTER_PCT:
        wall_state = 1
    elif wall_fill_pct <= WALL_FILL_EXIT_PCT:
        wall_state = 0

    if wall_blob is not None:
        img.draw_rectangle(wall_blob.rect, color=BLUE_WALL_COLOR, thickness=2)
        img.draw_string(
            (wall_blob.x, max(wall_blob.y - 12, 0)),
            "WALL %d%% %d" % (wall_fill_pct, wall_state),
            color=BLUE_WALL_COLOR,
            scale=1,
        )
    print(wall_state, wall_fill_pct)
    send_wall(wall_state, wall_fill_pct)


    if detection_mode == 1:
        green_led.off()
        blue_led.on()
        # Unload area: one large merged region, no cube shape filtering.
        object_type, color_name, threshold, box_color, known_width_mm = BLACK_CONFIG
        for blob in img.find_blobs(
            [threshold],
            roi=roi,
            pixels_threshold=BLACK_PIXELS_THRESHOLD,
            area_threshold=BLACK_PIXELS_THRESHOLD,
            merge=True,
        ):
            candidates.append((
                blob,
                object_type,
                color_name,
                box_color,
                known_width_mm,
            ))
    elif detection_mode == 0 and not lock_pause_active:
        blue_led.off()
        green_led.on()
        black_area_blob = None
        black_threshold = BLACK_CONFIG[2]
        for black_blob in img.find_blobs(
            [black_threshold],
            roi=roi,
            pixels_threshold=BLACK_PIXELS_THRESHOLD,
            area_threshold=BLACK_PIXELS_THRESHOLD,
            merge=True,
        ):
            if black_area_blob is None or black_blob.pixels > black_area_blob.pixels:
                black_area_blob = black_blob

        # Detect each color separately, then select one global largest valid blob.
        for object_type, color_name, threshold, box_color, known_width_mm in color_configs:
            blobs = img.find_blobs(
                [threshold],
                roi=roi,
                pixels_threshold=PIXELS_THRESHOLD,
                area_threshold=AREA_THRESHOLD,
                merge=False,
            )

            for blob in blobs:
                w = blob.w
                h = blob.h

                if w < MIN_BLOB_WIDTH or h < MIN_BLOB_HEIGHT:
                    continue

                aspect_ratio = w / h
                if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                    continue

                if black_area_blob is not None:
                    overlap_ratio = box_overlap_ratio(blob, black_area_blob)
                    if overlap_ratio >= BLACK_OVERLAP_REJECT_RATIO:
                        print("Ignore", color_name, "in black area")
                        continue

                candidates.append((
                    blob,
                    object_type,
                    color_name,
                    box_color,
                    known_width_mm,
                ))
    elif detection_mode == 0:
        blue_led.off()
        green_led.on()
    else:
        green_led.off()
        blue_led.off()

    if detection_mode == 1:
        target = largest_blob(candidates)
    elif detection_mode == 0:
        if lock_pause_active:
            target = None
        elif locked_target is None:
            target = largest_blob(candidates)
            if target is not None:
                blob = target[0]
                locked_target = (target[1], blob.cx, blob.cy, blob.w, blob.h)
        else:
            target = matching_locked_blob(candidates, locked_target)
            if target is None:
                locked_target = None
                lock_pause_started_ms = time.ticks_ms()
            else:
                blob = target[0]
                locked_target = (target[1], blob.cx, blob.cy, blob.w, blob.h)
    else:
        target = None

    if target is None:
        print("No target found, mode", detection_mode, "pause", lock_pause_active)
        if detection_mode == 1:
            send_arrival(0, 0)
        send_target(0, 0, 0, 0)
        continue

    blob, object_type, color_name, box_color, known_width_mm = target
    x = blob.x
    y = blob.y
    w = blob.w
    h = blob.h
    cx = blob.cx
    cy = blob.cy
    black_corners = None

    if object_type == BLACK_TYPE:
        # Use the minimum rotated rectangle and the intersection of its
        # diagonals, so a slanted black unload area is not centered from the
        # axis-aligned bounding box.
        try:
            black_corners = blob.min_corners
            if callable(black_corners):
                black_corners = black_corners()
            if len(black_corners) == 4:
                cx, cy = diagonal_intersection(black_corners, blob.cx, blob.cy)
            else:
                black_corners = None
        except Exception:
            black_corners = None
            cx = blob.cx
            cy = blob.cy

        # Use actual threshold-matched pixels, not the axis-aligned w*h box.
        # This excludes non-black background inside a slanted bounding box.
        black_fill_pct = min(
            100,
            int(100 * blob.pixels / (IMAGE_WIDTH * IMAGE_HEIGHT)),
        )
        black_arrived = 1 if black_fill_pct > BLACK_ARRIVAL_FILL_PCT else 0
        send_arrival(black_arrived, black_fill_pct)
        print("BLACK", black_arrived, black_fill_pct)

        # A floor region fills the frame when close, so its width saturates.
        # Use the gap between its near edge and the image bottom instead:
        # the gap shrinks to zero as the car drives onto the area.
        gap_px = IMAGE_HEIGHT - (y + h)
        if gap_px < 0:
            gap_px = 0
        distance_cm = int(gap_px * BLACK_NEAR_EDGE_CM_PER_PX)
        distance_mm = distance_cm * 10.0
    else:
        distance_mm = (FOCAL_LENGTH_PX * known_width_mm) / w
        distance_cm = int(distance_mm / 10.0)

    if black_corners is not None:
        for corner_index in range(4):
            next_index = (corner_index + 1) % 4
            x1, y1 = black_corners[corner_index]
            x2, y2 = black_corners[next_index]
            img.draw_line((x1, y1, x2, y2), color=box_color, thickness=2)
    else:
        img.draw_rectangle(blob.rect, color=box_color, thickness=2)
    img.draw_cross((cx, cy), color=box_color, size=5, thickness=2)
    label_y = max(y - 12, 0)
    if object_type == BLACK_TYPE:
        label = "%s %d%%" % (color_name, black_fill_pct)
    else:
        label = "%s %.1fmm" % (color_name, distance_mm)
    img.draw_string((x, label_y), label, color=box_color, scale=1)

    print("TX", object_type, cx, cy, distance_cm)
    send_target(object_type, cx, cy, distance_cm)
