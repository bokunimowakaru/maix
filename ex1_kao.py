###############################################################################
# Example 1 AIカメラが顔検出した位置をログ出力する
#                        for Sipeed M1n Module AI Development Kit based on K210
#
#                        Copyright (c) 2021 Wataru KUNINO https://bokunimo.net/
###############################################################################
# (参考文献) https://github.com/sipeed/MaixPy_scripts/
#                        blob/master/machine_vision/face_find/demo_find_face.py

import sensor, image
import KPU as kpu

# カメラ設定
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(True)
sensor.set_vflip(False)
sensor.run(1)

# AI設定
anchors = (1.889, 2.525, 2.947, 3.941, 4.0, 5.366, 5.155, 6.923, 6.718, 9.01)
task = kpu.load(0x300000)
kpu.init_yolo2(task, 0.3, 0.1, len(anchors)//2, anchors)

while(True):
    img = sensor.snapshot()
    objects = kpu.run_yolo2(task, img)
    n = 0
    if objects:
        n = len(objects)
        for obj in objects:
            img.draw_rectangle(obj.rect())
            img.draw_string(obj.x(), obj.y(), str(obj.value()))
            print(obj.rect(), end=', ')
        print()
    img.draw_string(0, 200, 'n=' + str(n), scale=2)
