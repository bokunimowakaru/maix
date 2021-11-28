###############################################################################
# Example 2 顔検出 LED
#                        for Sipeed M1n Module AI Development Kit based on K210
#
#                        Copyright (c) 2021 Wataru KUNINO https://bokunimo.net/
###############################################################################
# (参考文献) https://github.com/sipeed/MaixPy_scripts/
#                        blob/master/machine_vision/face_find/demo_find_face.py
#                        blob/master/hardware/demo_gpio_led.py
import sensor, image
import KPU as kpu
from Maix import GPIO

# LED設定
led = GPIO(GPIO.GPIO7, GPIO.OUT)

# カメラ設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(False)
sensor.set_vflip(True)
sensor.run(1)

# AI設定
anchors = (1.889, 2.525, 2.947, 3.941, 4.0, 5.366, 5.155, 6.923, 6.718, 9.01)
task = None
task = kpu.load(0x300000)
kpu.init_yolo2(task, 0.3, 0.1, len(anchors)//2, anchors)

while(not task is None):
    img = sensor.snapshot()
    objects = kpu.run_yolo2(task, img)
    n = 0
    if objects:
        led.value(1)
        n = len(objects)
        for obj in objects:
            img.draw_rectangle(obj.rect())
            img.draw_string(obj.x(), obj.y(), str(obj.value()))
            print(obj.rect(), end=', ')
        print()
    else:
        led.value(0)
    img.draw_string(0, 200, 'n=' + str(n), scale=2)
kpu.deinit(task)
