# OpenMV -> STM32 USART2 visual transmission test.
# Frame: V,color,cx,cy,distance_cm\n
# color: 0=none, 1=red, 2=yellow.

import sensor
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
    candidates = []
    led = LED(2)
    led.on()

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
        print("No red or yellow block found")
        send_target(0, 0, 0, 0)
        continue

    blob, object_type, color_name, box_color, known_width_mm = target
    x = blob.x()
    y = blob.y()
    w = blob.w()
    h = blob.h()
    cx = blob.cx()
    cy = blob.cy()

    distance_mm = (FOCAL_LENGTH_PX * known_width_mm) / w
    distance_cm = int(distance_mm / 10.0)

    img.draw_rectangle(blob.rect(), color=box_color, thickness=2)
    img.draw_cross((cx, cy), color=box_color, size=5, thickness=2)
    label_y = max(y - 12, 0)
    label = "%s %.1fmm" % (color_name, distance_mm)
    img.draw_string((x, label_y), label, color=box_color, scale=1)

    print("TX", object_type, cx, cy, distance_cm)
    send_target(object_type, cx, cy, distance_cm)
