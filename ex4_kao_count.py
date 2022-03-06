################################################################################
# Example 4 AIカメラによる来場者カウンタ for Sipeed Maix Dock
#
# 7番ピン (Wi-Fi TX端子) から累積来場者数を出力します。
#
#                   Copyright (c) 2021-2022 Wataru KUNINO https://bokunimo.net/
################################################################################

import sensor, lcd                                  # カメラsensor,液晶lcdの組込
import KPU as kpu                                   # AI演算ユニットKPUの組込
from machine import UART                            # UARTモジュールの組込
from fpioa_manager import fm                        # FPIOA管理モジュールの組込

BufHist_N = 10                                      # 計測用バッファ数
Det_N = 3                                           # うち検出判定用バッファ数
Det_Thresh = 0.2                                    # 検出閾値(小さいほど緩い)
MotionFact = 5.0                                    # 動き判定係数(大ほど緩い)
ExtentFact = 2.0                                    # 遠近判定係数(大ほど緩い)
ErrorFact = 0.2                                     # 誤判定係数(大ほど緩い)

def det_filter(obj, buf, start, end):               # バッファとの一致レベル計算
    level = 0.0                                     # 一致レベルを保持する変数
    if start >= end:                                # 0除算の回避
        return 0.0                                  # 0を応答
    for i in range(start, end):                     # バッファ1件ごとの処理
        for j in range(len(buf[i])):                # 検知人数ごとの処理
            # ↓検知位置が、過去に検知した範囲の近い場所かどうかを確認
            if obj.x() + (0.5 - MotionFact) * obj.w() < buf[i][j][0] and\
               obj.x() + (0.5 + MotionFact) * obj.w() > buf[i][j][0] and\
               obj.y() + (0.5 - MotionFact) * obj.h() < buf[i][j][1] and\
               obj.y() + (0.5 + MotionFact) * obj.h() > buf[i][j][1] and\
               obj.w() - ExtentFact * obj.w() < buf[i][j][2] and\
               obj.w() + ExtentFact * obj.w() > buf[i][j][2] and\
               obj.h() - ExtentFact * obj.h() < buf[i][j][3] and\
               obj.h() + ExtentFact * obj.h() > buf[i][j][3]:
                level += buf[i][j][4]               # 一致レベルを加算
                break                               # 同一データでの重複加算防止
    return level / (end - start)                    # 比率に変換

fm.register(7, fm.fpioa.UART1_TX, force=True)       # ポート7をUART1_TXに割当
uart = UART(UART.UART1, 115200, 8, 0, 1)            # UART1のオブジェクトuart
uart.write('0,0\n')                                 # UART送信(検知0,来場者数0)

lcd.init()                                          # LCDの初期化
sensor.reset()                                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565)                 # 色設定(白黒時GRAYSCALE)
sensor.set_framesize(sensor.QVGA)                   # 解像度設定(QVGA:320x240)
sensor.set_vflip(True)                              # カメラ画像の上下反転設定
sensor.set_hmirror(True)                            # カメラ画像の左右反転設定

# AIの設定(face_model_at_0x300000.kfpkg用)
anchors = (1.889, 2.525, 2.947, 3.941, 4.0, 5.366, 5.155, 6.923, 6.718, 9.01)
task = kpu.load(0x300000)
kpu.init_yolo2(task, Det_Thresh, 0.1, len(anchors)//2, anchors)

buf = list()                                        # 撮影ごとの検知位置バッファ
count = 0                                           # 来場者数

while(True):                                        # 永久ループ
    img = sensor.snapshot()                         # 撮影した写真をimgに代入
    objects = kpu.run_yolo2(task, img)              # 写真img内の顔検出を実行
    n = 0                                           # 検出件数を保持する変数n
    objs_rect = list()                              # 1撮影分の検出データ保持用
    if objects:                                     # 1人以上の顔を検出したとき
        n = len(objects)                            # 検出件数を数値変数nに代入
        for obj in objects:                         # 個々の検出結果ごとの処理
            img.draw_rectangle(obj.rect())          # 検出範囲をimgに追記
            img.draw_string(obj.x(), obj.y(), str(obj.value())) # 文字列を追記
            objs_rect.append([int(obj.x()+obj.w()/2),\
                              int(obj.y()+obj.h()/2),\
                              obj.w(),obj.h(),obj.value()])
        if len(buf) >=  BufHist_N:                  # バッファ数を満たすとき
            vals = list()                           # 検知レベル保持用(ログ用)
            for obj in objects:                     # 個々の検出結果ごとの処理
                det = det_filter(obj, buf, BufHist_N - Det_N, BufHist_N)
                # ↑直近のbufに顔が含まれているかどうかを確認(det:検知確認用)
                ndet = None                         # 検知レベル(非検知確認用)
                if det >= (1 + ErrorFact) * Det_Thresh:
                    ndet = det_filter(obj, buf, 0, BufHist_N - Det_N)
                    # ↑古いbufには含まれていないことを確認(ndet:非検知確認用)
                    if ndet <= ErrorFact:           # 含まれていないとき
                        count += 1                  # 来場者数としてカウント
                        uart.write(str(n)+',')      # 現在の検知数をシリアル出力
                        uart.write(str(count)+'\n') # 来場者数をシリアル出力
                        buf.clear()                 # バッファをクリア
                vals.append((det,ndet))             # 各レベルを保持(ログ用)
            print(vals)                             # 検知レベルを表示
    buf.append(objs_rect)                           # バッファに顔位置を保存
    if len(buf) > BufHist_N:                        # 最大容量を超過したとき
        del buf[0]                                  # 最も古いデータ1件を消去
    img.draw_string(0, 180, 'n=' + str(n), scale=3) # 検出件数をimgに追記する
    img.draw_string(0, 210, 'count=' + str(count), scale=3) # 検出件数を追記する
    lcd.display(img)                                # imgをLCDに表示

################################################################################
# UART出力例 現在の検知人数と、累計の来場者数をシリアル出力する
################################################################################
'''
0,0  <---- 起動時
1,1  <---- 一人目を検出し、カンマの左側に現在の検出人数1が表示された
2,2  <---- 二人目を検出し、検出人数2が表示された
1,3  <---- 三人目を検出し、カンマの右側には累計数が表示された
1,4
1,5
2,6  <---- このあとも累計数(カンマの右側の数値)は検出するたびに1ずつ増えてゆく
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
