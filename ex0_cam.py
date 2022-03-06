################################################################################
# Example 0 カメラの画像を表示する for Sipeed Maix Dock
#
#                    Copyright (c) 2021-2022 Wataru KUNINO https://bokunimo.net/
################################################################################
# MaixPy IDE を使った AIカメラ のプログラム作成方法の確認用のサンプルです。
# デバイスの動作確認と基本機能のプログラミングに使用します。
#
# 1. Sipeed Maix Dock を PC に接続してください。
# 2. MaixPy IDE の[ツール]メニュー内のSelect Boardから Sipeed Maix Dock を選択。
# 3. 画面の左下から2つ目の接続ボタン(緑色)をクリックしてボードに接続します。
#    (複数のシリアル・ポートが存在する場合は、COMポートを選択してください)
# 4. 画面の左下の実行ボタン(3の操作で緑色に変化する)をクリックしてください。

import sensor, lcd                                  # カメラsensor,液晶lcdの組込

lcd.init()                                          # LCDの初期化
sensor.reset()                                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565)                 # 色設定(白黒時GRAYSCALE)
sensor.set_framesize(sensor.QVGA)                   # 解像度設定(QVGA:320x240)
sensor.set_vflip(True)                              # カメラ画像の上下反転設定
sensor.set_hmirror(True)                            # カメラ画像の左右反転設定

while(True):                                        # 永久ループ
    img = sensor.snapshot()                         # 撮影した写真をimgに代入
    img.draw_string(5,0,"Hello, World!",0x0000,3)   # 文字列 Hello～ をimgに追記
    img.draw_string(15,210,"bokunimo.net",0xFFFF,3) # 文字列 bokunimo～ を追記
    lcd.display(img)                                # 以上の結果をLCDに表示

################################################################################
# (参考文献)
# ・demo_fps_display.py
#   (https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/)
# ・helloworld.py (MaixPy IDE 0.2.5)
#   (https://dl.sipeed.com/shareURL/MAIX/MaixPy/ide/)
