# OpenMV -> STM32 USART2 visual protocol.
# Frame: V,detected,x_offset_px,y_offset_px,distance_cm,object_type\n

import sensor
import image
import time
from pyb import LED, UART

# UART3 pins on OpenMV H7 Plus: P4=TX, P5=RX.
# Connect P4 to STM32 PA3 (USART2_RX), P5 to STM32 PA2 (USART2_TX),
# and connect both GND pins.
uart = UART(3, 115200, bits=8, parity=None, stop=1, timeout_char=20)

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
RED_TYPE = 1
YELLOW_TYPE = 2


def send_target(detected, x_offset_px, y_offset_px, distance_cm, object_type):
    uart.write("V,%d,%d,%d,%d,%d\n" % (
        detected,
        x_offset_px,
        y_offset_px,
        distance_cm,
        object_type,
    ))


def largest_blob(blobs):
    if not blobs:
        return None

    target = blobs[0]
    for blob in blobs:
        if blob.pixels() > target.pixels():
            target = blob
    return target

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

green_led = LED(2)
for _ in range(3):
    green_led.on()
    time.sleep_ms(200)
    green_led.off()
    time.sleep_ms(200)

# ROI: (x, y, w, h)
roi = (0, 0, 640, 480)

# LAB threshold: (L_min, L_max, a_min, a_max, b_min, b_max)
# Format: (object_type, name, LAB threshold, drawing color, real object width in millimeters)
# Replace each width with the measured width of that color's object.
color_configs = (
    (RED_TYPE, "RED", (20, 75, 20, 127, -20, 70), (255, 0, 0), 25.0),
    (YELLOW_TYPE, "YELLOW", (43, 100, -29, 10, 31, 95), (255, 255, 0), 30.0),
)

# Smaller values make detection more sensitive, but may also detect noise.
PIXELS_THRESHOLD = 75
AREA_THRESHOLD = 75

# Shape filters for cube/cylinder side views.
# Reject thin wires, tiny regions, and sparse color noise.
MIN_ASPECT_RATIO = 0.5
MAX_ASPECT_RATIO = 2.0
MIN_BLOB_WIDTH = 4
MIN_BLOB_HEIGHT = 4
# Focal length in pixels. This is the calibration constant.
# For QVGA, a typical value is around 300~700 depending on lens and setup.
# Tune this by measuring a known distance once and adjusting.
FOCAL_LENGTH_PX = 370

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    detected_count = 0

    # Detect each color separately so nearby colors are not merged together.
    for object_type, color_name, threshold, box_color, known_width_mm in color_configs:
        blobs = img.find_blobs(
            [threshold],
            roi=roi,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=False,
        )

        for blob in blobs:
            x = blob.x
            y = blob.y
            w = blob.w
            h = blob.h
            cx = blob.cx
            cy = blob.cy
            width_px = w

            if w < MIN_BLOB_WIDTH or h < MIN_BLOB_HEIGHT:
                continue

            aspect_ratio = w / h
            if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                continue

            # A solid block fills most of its bounding box; a wire usually does not.

            distance_mm = (FOCAL_LENGTH_PX * known_width_mm) / width_px
            distance_cm = int(distance_mm / 10.0)
            detected_count += 1

            img.draw_rectangle((x, y, w, h), color=box_color, thickness=2)
            bottom_center_x = x + (w // 2)
            bottom_center_y = min(y + h - 1, 239)
            img.draw_cross(
                (bottom_center_x, bottom_center_y),
                color=box_color,
                size=5,
                thickness=2,
            )

            label_y = max(y - 12, 0)
            label = "%s %.1fmm" % (color_name, distance_mm)
            img.draw_string((x, label_y), label, color=box_color, scale=1)

            x_offset_px = cx - (IMAGE_WIDTH // 2)
            y_offset_px = cy - (IMAGE_HEIGHT // 2)
            print(color_name, x_offset_px, y_offset_px, distance_cm)
            send_target(1, x_offset_px, y_offset_px, distance_cm, object_type)

    if detected_count == 0:
        print("No red or yellow block found")
        send_target(0, 0, 0, 0, 0)
