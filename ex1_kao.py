################################################################################
# Example 1 AIカメラが顔検出した位置をログ出力する for Sipeed Maix Dock
# 
# ・AI演算用のKPU(Neural Network Processing Unit)を使って顔検出を行います。
# ・顔検出した範囲をLCDに表示するとともに、検出位置をログ出力します。
#
#                   Copyright (c) 2021-2022 Wataru KUNINO https://bokunimo.net/
################################################################################
# KModel を使った顔検出機能の動作確認と、基本プログラミングに使用します。
# 実行前に下記の手順で、モデルをダウンロードしてボードに書き込んでください。
#
# 1. 顔検出用の機械学習モデルを下記のSipeed社のサイトからダウンロードします。
#       https://dl.sipeed.com/shareURL/MAIX/MaixPy/model
#       face_model_at_0x300000.kfpkg
# 2. ダウンロードしたkfpkgファイルを、K-Flashを使ってボードに書き込みます。
#       https://github.com/kendryte/kendryte-flash-windows/releases
#       K-Flash v0.4.1
# 3. MaixPy IDE をボードに接続し、本プログラムを実行します(ex0_cam参照)。

import sensor, lcd                                  # カメラsensor,液晶lcdの組込
import KPU as kpu                                   # AI演算ユニットKPUの組込

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
    if objects:                                     # 1件以上検出したとき
        n = len(objects)                            # 検出件数を数値変数nに代入
        for obj in objects:                         # 個々の検出結果ごとの処理
            img.draw_rectangle(obj.rect())          # 検出範囲をimgに追記
            img.draw_string(obj.x(), obj.y(), str(obj.value())) # 文字列を追記
            print(obj.rect(), end=', ')             # 検出結果をログ出力
        print()                                     # 改行をログ出力
    img.draw_string(0, 210, 'n=' + str(n), scale=3) # 検出件数をimgに追記する
    lcd.display(img)                                # imgをLCDに表示

################################################################################
# 参考文献
################################################################################
#・demo_find_face.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/face_find/)
