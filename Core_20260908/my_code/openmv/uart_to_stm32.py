# OpenMV -> STM32 USART2视觉协议。
# 帧: V,detected,x_offset_px,y_offset_px,distance_cm,object_type\n
# STM32命令: M,0\n=物块模式，M,1\n=黑色卸货区模式。
# 将本文件复制到OpenMV并命名为main.py。

import sensor
import image
import time
from pyb import UART

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
BLOCK_THRESHOLD = (81, 99, -28, -7, -33, 93)
# 必须在实际比赛场地用Threshold Editor重新标定。
BLACK_THRESHOLD = (0, 35, -20, 20, -20, 20)

BLOCK_TYPE = 1
BLACK_AREA_TYPE = 3
BLOCK_WIDTH_CM = 8.0
BLACK_AREA_WIDTH_CM = 20.0
FOCAL_LENGTH_PX = 480.0

uart = UART(3, 115200, timeout_char=20)
detection_mode = 0

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)


def read_stm32_mode():
    global detection_mode
    while uart.any():
        line = uart.readline()
        if line:
            try:
                text = line.decode().strip()
                if text == "M,0":
                    detection_mode = 0
                elif text == "M,1":
                    detection_mode = 1
            except Exception:
                pass


def largest_blob(blobs):
    if not blobs:
        return None
    result = blobs[0]
    for blob in blobs:
        if blob.pixels > result.pixels:
            result = blob
    return result


while True:
    read_stm32_mode()
    img = sensor.snapshot()

    if detection_mode == 0:
        threshold = BLOCK_THRESHOLD
        object_type = BLOCK_TYPE
        known_width_cm = BLOCK_WIDTH_CM
    else:
        threshold = BLACK_THRESHOLD
        object_type = BLACK_AREA_TYPE
        known_width_cm = BLACK_AREA_WIDTH_CM

    target = largest_blob(img.find_blobs(
        [threshold],
        roi=(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT),
        pixels_threshold=200,
        area_threshold=200,
        merge=True,
    ))

    if target is None:
        uart.write("V,0,0,0,0,0\n")
        time.sleep_ms(50)
        continue

    size_px = max(target.w, target.h)
    if size_px <= 0:
        uart.write("V,0,0,0,0,0\n")
        time.sleep_ms(50)
        continue

    distance_cm = int(FOCAL_LENGTH_PX * known_width_cm / size_px)
    x_offset_px = int(target.cx - IMAGE_WIDTH // 2)
    y_offset_px = int(target.cy - IMAGE_HEIGHT // 2)

    img.draw_rectangle((target.x, target.y, target.w, target.h),
                       color=(255, 0, 0), thickness=2)
    img.draw_cross((target.cx, target.cy),
                   color=(0, 255, 0), size=5, thickness=2)
    uart.write("V,1,%d,%d,%d,%d\n" %
               (x_offset_px, y_offset_px,
                distance_cm, object_type))
    time.sleep_ms(50)
