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

BufHist_N = 10      # 2～                           # 動き検出用バッファ数
Det_N = 3           # 1～BufHist_N                  # うち検出判定用
Det_Thresh = 0.2                                    # 検出閾値(小さいほど緩い)
MotionFact = 2.0                                    # 動き判定係数(大ほど緩い)
ExtentFact = 2.0                                    # 遠近判定係数(大ほど緩い)
ErrorFact = 0.2                                     # 誤判定係数(大ほど緩い)

def det_filter(obj, buf):                           # バッファとの一致レベル計算
    if len(buf) < BufHist_N:                        # バッファ不足時
        return(0,0)                                 # 0を応答
    det = 0.0                                       # 検知確認用の一致レベル
    ndet = 0.0                                      # 非検知確認用の一致レベル
    x = obj.x()                                     # 現在の顔位置座標xを保持
    y = obj.y()                                     # 現在の顔位置座標yを保持
    w = obj.w()                                     # 現在の顔サイズ幅wを保持
    h = obj.h()                                     # 現在の顔サイズ高hを保持
    for i in range(BufHist_N-1,-1,-1):              # バッファ1件ごとの処理
        for j in range(len(buf[i])):                # 検知人数ごとの処理
            # ↓検知位置が、過去に検知した範囲の近い場所かどうかを確認
            if x + (0.5 - MotionFact) * w < buf[i][j][0] + buf[i][j][2]/2 and\
               x + (0.5 + MotionFact) * w > buf[i][j][0] + buf[i][j][2]/2 and\
               y + (0.5 - MotionFact) * h < buf[i][j][1] + buf[i][j][3]/2 and\
               y + (0.5 + MotionFact) * h > buf[i][j][1] + buf[i][j][3]/2 and\
               w - ExtentFact * w < buf[i][j][2] and\
               w + ExtentFact * w > buf[i][j][2] and\
               h - ExtentFact * h < buf[i][j][3] and\
               h + ExtentFact * h > buf[i][j][3]:
                if i >= BufHist_N - Det_N:          # 直近Det_Nのバッファ処理時
                    det += buf[i][j][4]             # 一致レベルをdetに加算
                else:                               # Det_Nより古いバッファ処理
                    ndet += buf[i][j][4]            # 一致レベルをndetに加算
                x = buf[i][j][0]                    # 発見した顔位置にxを更新
                y = buf[i][j][1]                    # 発見した顔位置にyを更新
                w = buf[i][j][2]                    # 発見した顔サイズ幅wに更新
                h = buf[i][j][3]                    # 発見した顔サイズ高hに更新
                break                               # 同一データでの重複加算防止
    return (det/Det_N, ndet/(BufHist_N - Det_N))    # detとndetを比率にして応答

fm.register(7, fm.fpioa.UART1_TX, force=True)       # IO7ピンをUART1_TXに割当
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
            objs_rect.append([obj.x(), obj.y(), obj.w(), obj.h(), obj.value()])
        if len(buf) >=  BufHist_N:                  # バッファ数を満たすとき
            vals = list()                           # 検知レベル保持用(ログ用)
            for obj in objects:                     # 個々の検出結果ごとの処理
                (det,ndet) = det_filter(obj, buf)
                # det :直近のbufに顔が含まれているかどうかを確認(検知確認用)
                # ndet:古いbufに顔が含まれていないことを確認(ndet:非検知確認用)
                if det >= (1 + ErrorFact) * Det_Thresh and ndet <= ErrorFact:
                    count += 1                      # 来場者数としてカウント
                    uart.write(str(n)+',')          # 現在の検知数をシリアル出力
                    uart.write(str(count)+'\n')     # 来場者数をシリアル出力
                    for i in range(BufHist_N):      # 非検知確認用の区間
                        buf[i] = objs_rect          # バッファを最新値で上書き
                vals.append((det,ndet))             # 各レベルを保持(ログ用)
            print(vals)                             # 検知レベルをログ表示
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
# ログ出力例 検知した顔の数だけ,その検知レベルを直近と過去に分けて出力する
################################################################################
'''
1人の顔を見つけてレベルが上がる様子
[(0.0, 0.0)]                非検知状態
[(0.1666667, 0.0)]          直近に顔を検出
[(0.3521199, 0.0)]          数値が増大(閾値を超過すると来場者としてカウント)
[(0.278995, 0.1943613)]     古いバッファの数値も増大(新たにカウントしない)
[(0.4079065, 0.3264704)]    以下、人がいるだけではカウントしない
[(0.3895914, 0.3343197)]
[(1人目直近, 1人目古い)]

2人の顔を検知しているときのデータ例
[(0.4628983, 0.5785333), (0.5545204, 0.4147943)]
[(0.5187867, 0.5467318), (0.4320156, 0.3835688)]
[(0.4816848, 0.5233659), (0.4503308, 0.3205416)]
[(1人目直近, 1人目古い), (2人目直近, 2人目古い)]
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
