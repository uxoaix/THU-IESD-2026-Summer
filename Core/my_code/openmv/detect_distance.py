# OpenMV -> STM32 USART2 visual transmission test.
# Frame: V,color,cx,cy,distance_cm\n
# color: 0=none, 1=red, 2=yellow, 3=black unload area.
# STM32 command: M,0\n=block mode, M,1\n=black unload area mode.

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

# 0 = search red/yellow blocks, 1 = search the black unload area.
detection_mode = 0


def read_stm32_mode():
    """Apply the latest M,0 / M,1 command sent by the STM32."""
    global detection_mode
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


def send_target(color, cx, cy, distance_cm):
    uart.write("V,%d,%d,%d,%d\n" % (
        color,
        cx,
        cy,
        distance_cm,
    ))


def largest_blob(candidates):
    """Return (blob, color config) having the largest pixel count."""
    target = None
    for candidate in candidates:
        if target is None or candidate[0].pixels() > target[0].pixels():
            target = candidate
    return target

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
BLACK_CONFIG = (BLACK_TYPE, "BLACK", (0, 35, -20, 20, -20, 20), (0, 0, 255), 400.0)
# The area is a large floor region, so it needs a bigger blob and merging.
BLACK_PIXELS_THRESHOLD = 300
# Reported distance per pixel of gap below the area's near edge.
# Increase it if the car stops too early, decrease it if it overshoots.
BLACK_NEAR_EDGE_CM_PER_PX = 0.3

# Smaller values make detection more sensitive, but may also detect noise.
# QVGA blob area is a quarter of the VGA area for the same object.
PIXELS_THRESHOLD = 20
AREA_THRESHOLD = 20

# Shape filters for cube/cylinder side views.
# Reject thin wires, tiny regions, and sparse color noise.
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 2.0
MIN_BLOB_WIDTH = 4
MIN_BLOB_HEIGHT = 4
# Focal length in pixels. This is the calibration constant.
# It scales with resolution, so QVGA uses half of the VGA value.
# Tune this by measuring a known distance once and adjusting.
FOCAL_LENGTH_PX = 185

clock = time.clock()

while True:
    clock.tick()
    read_stm32_mode()
    img = sensor.snapshot()
    candidates = []


    if detection_mode != 0:
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
    else:
        green_led.on()
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
                w = blob.w()
                h = blob.h()

                if w < MIN_BLOB_WIDTH or h < MIN_BLOB_HEIGHT:
                    continue

                aspect_ratio = w / h
                if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                    continue

                candidates.append((
                    blob,
                    object_type,
                    color_name,
                    box_color,
                    known_width_mm,
                ))

    target = largest_blob(candidates)
    if target is None:
        print("No target found, mode", detection_mode)
        send_target(0, 0, 0, 0)
        continue

    blob, object_type, color_name, box_color, known_width_mm = target
    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    cx = blob.cx()
    cy = blob.cy()

    if object_type == BLACK_TYPE:
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

    img.draw_rectangle(blob.rect(), color=box_color, thickness=2)
    img.draw_cross((cx, cy), color=box_color, size=5, thickness=2)
    label_y = max(y - 12, 0)
    label = "%s %.1fmm" % (color_name, distance_mm)
    img.draw_string((x, label_y), label, color=box_color, scale=1)

    print("TX", object_type, cx, cy, distance_cm)
    send_target(object_type, cx, cy, distance_cm)
