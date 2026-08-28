# OpenMV monocular distance estimation example
# Distance is estimated from the size of the tracked blob and a known object width.
# The formula is: distance(cm) = focal_length_px * known_object_width_cm / object_width_px
# You need to calibrate 'KNOWN_WIDTH_CM' and 'FOCAL_LENGTH_PX' for your camera and target.

import sensor
import image
import time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# ROI: (x, y, w, h)
roi = (0, 0, 320, 240)

# LAB threshold: (L_min, L_max, a_min, a_max, b_min, b_max)
# Replace with your actual target color threshold.
thresholds = [(81, 99, -28, -7, -33, 93)]

# ---- Distance calibration parameters ----
# Real size of the target object in centimeters.
KNOWN_WIDTH_CM = 8.0

# Focal length in pixels. This is the calibration constant.
# For QVGA, a typical value is around 300~700 depending on lens and setup.
# Tune this by measuring a known distance once and adjusting.
FOCAL_LENGTH_PX = 480

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
        # Choose the biggest blob as the target.
        # On this OpenMV firmware, pixels is an integer property, not a callable method.
        target = blobs[0]
        for b in blobs:
            if b.pixels > target.pixels:
                target = b

        x = target.x
        y = target.y
        w = target.w
        h = target.h
        cx = target.cx
        cy = target.cy

        # Use the larger dimension as the object size in the image.
        size_px = max(w, h)

        # Distance estimation using similar triangles.
        # If size_px is zero, avoid division by zero.
        if size_px > 0:
            distance_cm = (FOCAL_LENGTH_PX * KNOWN_WIDTH_CM) / size_px
        else:
            distance_cm = 0

        # Draw the target box and center marker.
        # On this firmware, draw_rectangle expects a rect tuple: (x, y, w, h)
        img.draw_rectangle((x, y, w, h), color=(255, 0, 0), thickness=2)
        img.draw_cross((cx, cy), color=(0, 255, 0), size=5, thickness=2)

        # Show distance on the image.
        text = "D: %d cm" % int(distance_cm)
        img.draw_string((x, max(y - 20, 0)), text, color=(255, 255, 255), scale=2)

        print("blob size=%d px, distance=%d cm" % (size_px, int(distance_cm)))
    else:
        print("No blob found")
