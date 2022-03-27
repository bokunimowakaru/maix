################################################################################
# Example 3 AIカメラが顔検出した位置をシリアル出力する for Sipeed Maix Dock
#
# 7番ピン (Wi-Fi TX端子) から座標と大きさをUARTで送信します。
#
#                   Copyright (c) 2021-2022 Wataru KUNINO https://bokunimo.net/
################################################################################

import sensor, lcd                                  # カメラsensor,液晶lcdの組込
import KPU as kpu                                   # AI演算ユニットKPUの組込
from machine import UART                            # UARTモジュールの組込
from fpioa_manager import fm                        # FPIOA管理モジュールの組込

fm.register(7, fm.fpioa.UART1_TX, force=True)       # IO7ピンをUART1_TXに割当
uart = UART(UART.UART1, 115200, 8, 0, 1)            # UART1のオブジェクトuart
uart.write('Hello!\n')                              # UART送信

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
    n = 0                                           # 検出件数を保持する変数n
    s=''                                            # UART出力用の文字列変数s
    if objects:                                     # 1件以上検出したとき
        n = len(objects)                            # 検出件数を数値変数nに代入
        for obj in objects:                         # 個々の検出結果ごとの処理
            img.draw_rectangle(obj.rect())          # 検出範囲をimgに追記
            img.draw_string(obj.x(), obj.y(), str(obj.value())) # 文字列を追記
            if len(s) > 0:                          # s内に文字あり(for2回目～)
                s += ', '                           # 文字列sにカンマを追記
            s += str(obj.rect())                    # 文字列sに検出結果を追記
        print(s)                                    # 検出結果をログ出力
        uart.write(s + '\n')                        # 検出結果をUART出力
    img.draw_string(0, 190, 'n=' + str(n), scale=3) # 検出件数をimgに追記する
    img.draw_string(0, 218, s, scale=2)             # UART出力結果をimgに追記
    lcd.display(img)                                # imgをLCDに表示

################################################################################
# UART出力例
################################################################################
'''
(119, 49, 31, 40)
(250, 25, 24, 32), (119, 31, 31, 41)
(119, 23, 31, 41), (250, 23, 24, 33)
(249, 23, 24, 33), (119, 34, 30, 40)
(246, 35, 30, 41)
(119, 35, 30, 41), (246, 34, 30, 41)
(171, 25, 19, 26), (42, 36, 30, 41)
(169, 25, 24, 32)
'''

################################################################################
# 参考文献
################################################################################
#・demo_find_face.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/face_find/)
#・demo_uart_loop.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/hardware/demo_uart_loop.py
#・FPIOA情報
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/fpioa.html)
# (https://github.com/sipeed/MaixPy_scripts/blob/master/board/config_maix_dock.py)
#・KPU情報
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/kpu.html)
