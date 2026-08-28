# OpenMV monocular distance estimation example
# Distance is estimated from the size of the tracked blob and a known object width.
# The formula is: distance(mm) = focal_length_px * known_object_width_mm / object_width_px
# You need to calibrate each known width and 'FOCAL_LENGTH_PX' for your camera.

import sensor
import image
import time
from pyb import UART

# UART3 pins on OpenMV H7 Plus: P4=TX, P5=RX.
# Connect P4 to STM32 RX, P5 to STM32 TX, and connect both GND pins.
uart = UART(3, 115200, bits=8, parity=None, stop=1)

# One target per line: cx,cy,color_name,distance_mm\r\n
# Example: 156,108,RED,235.4
def send_target(center_x, center_y, color_name, distance_mm):
    message = "%d,%d,%s,%.1f\r\n" % (
        center_x,
        center_y,
        color_name,
        distance_mm,
    )
    uart.write(message)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# ROI: (x, y, w, h)
roi = (0, 0, 640, 480)

# LAB threshold: (L_min, L_max, a_min, a_max, b_min, b_max)
# Format: (name, LAB threshold, drawing color, real object width in millimeters)
# Replace each 25.0 with the measured width of that color's object.
color_configs = (
    ("RED", (20, 75, 20, 127, -20, 70), (255, 0, 0), 25.0),
    ("YELLOW", (43, 100, -29, 10, 31, 95), (255, 255, 0), 30.0),
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
    for color_name, threshold, box_color, known_width_mm in color_configs:
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

            print(
                    color_name,
                    cx,
                    cy,
            )
            send_target(cx, cy, color_name, distance_mm)

    if detected_count == 0:
        print("No red or yellow block found")
        send_target(0, 0, "NONE", 0.0)

