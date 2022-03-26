################################################################################
# Example 2 AIカメラが顔検出したときに LED を点灯する for Sipeed Maix Dock
#
#                   Copyright (c) 2021-2022 Wataru KUNINO https://bokunimo.net/
################################################################################

import sensor, lcd                                  # カメラsensor,液晶lcdの組込
import KPU as kpu                                   # AI演算ユニットKPUの組込
from Maix import GPIO                               # GPIOモジュールの組込
from fpioa_manager import fm                        # FPIOA管理モジュールの組込

fm.register(14, fm.fpioa.GPIO0, force=True)         # IO14ピンをGPIO0に割り当て
led_r = GPIO(GPIO.GPIO0, GPIO.OUT)                  # GPIO0のオブジェクトled_r
led_stat = ['On','Off']                             # led状態

lcd.init()                                          # LCDの初期化
sensor.reset()                                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565)                 # 色設定(白黒時GRAYSCALE)
sensor.set_framesize(sensor.QVGA)                   # 解像度設定(QVGA:320x240)
sensor.set_vflip(True)                              # カメラ画像の上下反転設定
sensor.set_hmirror(True)                            # カメラ画像の左右反転設定

# AIの設定(face_model_at_0x300000.kfpkg用)
anchors = (1.889, 2.525, 2.947, 3.941, 4.0, 5.366, 5.155, 6.923, 6.718, 9.01)
task = kpu.load(0x300000)
kpu.init_yolo2(task, 0.3, 0.1, len(anchors)//2, anchors)

while(True):                                        # 永久ループ
    img = sensor.snapshot()                         # 撮影した写真をimgに代入
    objects = kpu.run_yolo2(task, img)              # 写真img内の顔検出を実行
    if objects:                                     # 1件以上検出したとき
        img.draw_rectangle(objects[0].rect())       # 1件目の検出範囲をimgに追記
        led_r.value(led_stat.index('On'))           # LEDを点灯(GPIOをLレベルに)
    else:                                           # 検出しなかったとき
        led_r.value(led_stat.index('Off'))          # LEDを消灯(GPIOをHレベルに)
    img.draw_string(0,210,'LED='+led_stat[led_r.value()],scale=3) # 件数をimgに
    lcd.display(img)                                # imgをLCDに表示

################################################################################
# 参考文献
################################################################################
#・demo_find_face.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/face_find/)
#・demo_gpio_led.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/hardware/)
#・GPIO情報
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/gpio.html)
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/fpioa.html)
# (https://github.com/sipeed/MaixPy_scripts/blob/master/board/config_maix_dock.py)
#・KPU情報
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/kpu.html)
