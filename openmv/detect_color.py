# OpenMV color detection example
# Detect a color using a LAB threshold and draw a rectangle around it.

import sensor
import image
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking

# ROI must be a tuple: (x, y, w, h)
roi = (0, 0, 320, 240)

# LAB threshold: (L_min, L_max, a_min, a_max, b_min, b_max)
# Adjust these values to match your target color.
thresholds = [(71, 89, -18, 5, -27, 19)]

clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs(
        thresholds,
        roi=roi,
        pixels_threshold=200,
        area_threshold=200,
        merge=True,
    )

    if blobs:
        for blob in blobs:
            img.draw_detection(blob)
        print(blob.cx , blob.cy)
    else:
        print("No blob found")
