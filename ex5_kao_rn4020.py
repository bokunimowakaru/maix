################################################################################
# Example 5 BLE対応 AIカメラによる来場者カウンタ for Sipeed Maix Dock
#
# BLEモジュール RN4020を使って、累積来場者数を BLEビーコン 出力します。
#
#                   Copyright (c) 2021-2022 Wataru KUNINO https://bokunimo.net/
################################################################################

import sensor, lcd                                  # カメラsensor,液晶lcdの組込
import KPU as kpu                                   # AI演算ユニットKPUの組込
from machine import UART                            # UARTモジュールの組込
from Maix import GPIO                               # GPIOモジュールの組込
from fpioa_manager import fm                        # FPIOA管理モジュールの組込
from time import sleep                              # 待ち時間処理関数の組込

BufHist_N = 10      # 2～                           # 動き検出用バッファ数
Det_N = 3           # 1～BufHist_N                  # うち検出判定用
Det_Thresh = 0.2                                    # 検出閾値(小さいほど緩い)
MotionFact = 2.0                                    # 動き判定係数(大ほど緩い)
ExtentFact = 2.0                                    # 遠近判定係数(大ほど緩い)
ErrorFact = 0.2                                     # 誤判定係数(大ほど緩い)
ble_ad_id = 'CD00'                                  # BLEビーコン用ID(先頭bytes)

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

def rn4020(s = ''):                                 # BLE RN4020との通信用の関数
    if len(s) > 0:                                  # 変数sが1文字以上あるとき
        print('>',s)                                # 内容を表示
        uart.write(s + '\n')                        # コマンド送信
        sleep(0.1)                                  # 0.1秒の待ち時間処理
    while uart.any() > 0:                           # 受信バッファに文字がある時
        rx = uart.readline().decode()               # 受信データを変数sに代入
        print('<',rx.strip())                       # 受信結果を表示する
        sleep(0.1)                                  # 0.1秒の待ち時間処理

fm.register(7, fm.fpioa.UART1_TX, force=True)       # IO7ピンをUART1_TXに割当
fm.register(6, fm.fpioa.UART1_RX, force=True)       # IO8ピンをUART1_RXに割当
uart = UART(UART.UART1,115200,8,0,1,timeout=1000,read_buf_len=4096) # UART1設定
fm.register(14, fm.fpioa.GPIO0, force=True)         # IO14ピンをGPIO0に割り当て
fm.register(13, fm.fpioa.GPIO1, force=True)         # IO13ピンをGPIO1に割り当て
led_r = GPIO(GPIO.GPIO0, GPIO.OUT)                  # GPIO0のオブジェクトled_r
led_g = GPIO(GPIO.GPIO1, GPIO.OUT)                  # GPIO1のオブジェクトled_g
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
kpu.init_yolo2(task, Det_Thresh, 0.1, len(anchors)//2, anchors)

# BLE初期設定
rn4020('V')                                         # バージョン情報表示
rn4020('SF,2')                                      # 全設定の初期化
sleep(0.5)                                          # リセット待ち(1秒)
rn4020()                                            # 応答表示
rn4020('SR,20000000')                               # 機能設定:アドバタイジング
rn4020('SS,00000001')                               # サービス設定:ユーザ定義
rn4020('SN,RN4020_AICAM')                           # デバイス名:RN4020_AICAM
rn4020('R,1')                                       # RN4020を再起動
sleep(3)                                            # リセット待ち(送信開始)
rn4020('D')                                         # 情報表示
rn4020('Y')                                         # アドバタイジング停止

buf = list()                                        # 撮影ごとの検知位置バッファ
count = 0                                           # 来場者数

while(True):                                        # 永久ループ
    img = sensor.snapshot()                         # 撮影した写真をimgに代入
    objects = kpu.run_yolo2(task, img)              # 写真img内の顔検出を実行
    n = 0                                           # 検出件数を保持する変数n
    objs_rect = list()                              # 1撮影分の検出データ保持用
    if objects:                                     # 1人以上の顔を検出したとき
        led_g.value(led_stat.index('On'))           # 緑色LEDの点灯
        for obj in objects:                         # 個々の検出結果ごとの処理
            img.draw_rectangle(obj.rect())          # 検出範囲をimgに追記
            img.draw_string(obj.x(), obj.y(), str(obj.value())) # 文字列を追記
            objs_rect.append((obj.x(), obj.y(), obj.w(), obj.h(), obj.value()))
        if len(buf) >=  BufHist_N:                  # バッファ数を満たすとき
            objs_det = list()                       # 顔検出した結果の保持用
            count_new = 0                           # 新規の検出数
            for obj in objects:                     # 個々の検出結果ごとの処理
                (det,ndet) = det_filter(obj, buf)   # 関数det_filterを実行
                # det :直近のbufに顔が含まれているかどうかを確認(検知確認用)
                # ndet:古いbufに顔が含まれていないことを確認(ndet:非検知確認用)
                if det >= (1 + ErrorFact) * Det_Thresh: # 直近bufに含まれてる時
                    objs_det.append(objs_rect[objects.index(obj)]) # 顔位置保持
                    if ndet <= ErrorFact:           # 古いbufに含まれていない時
                        count_new += 1              # 新規検出数に1を加算
            n = len(objs_det)                       # 検出件数を数値変数nに代入
            if n > 0:                               # 顔検出結果がある時
                print(objs_det)                     # 顔検出結果をログ表示
            if count_new > 0:                       # 新しい顔検出結果がある時
                count += count_new                  # 来場者数(累計)に加算
                for i in range(BufHist_N):          # 非検知確認用の区間
                    buf[i] = objs_det               # バッファを最新値で上書き
                led_r.value(led_stat.index('On'))   # LEDを点灯(IOをLレベルに)
                count %= 65536                      # 65536以上でリセット
                s = ble_ad_id                       # BLEデータ生成(16進数変換)
                s += '{:02X}'.format(n%256)         # nの値を16進数に変換
                s += '{:02X}'.format(count%256)     # count下位バイトを16進数に
                s += '{:02X}'.format(count>>8)      # count上位バイトを16進数に
                rn4020('N,' + s)                    # ブロードキャスト情報に設定
                rn4020('A,0064,00C8')               # 0.1秒間隔0.2秒アドバタイズ
                sleep(0.1)                          # 0.1秒間の待ち時間処理
                rn4020('Y')                         # アドバタイジング停止
                led_r.value(led_stat.index('Off'))  # LED消灯(IOをHレベルに)
        led_g.value(led_stat.index('Off'))          # 緑色LEDの消灯
    buf.append(objs_rect)                           # バッファに顔位置を保存
    if len(buf) > BufHist_N:                        # 最大容量を超過したとき
        del buf[0]                                  # 最も古いデータ1件を消去
    img.draw_string(0, 180, 'n=' + str(n), scale=3) # 検出件数をimgに追記する
    img.draw_string(0, 210, 'count=' + str(count), scale=3) # 検出件数を追記する
    lcd.display(img)                                # imgをLCDに表示

################################################################################
# BLE受信例 現在の検知人数Numberと、累計の来場者数Countを受信した
################################################################################
# 下記のツールにてBLEを受信する
#
# ble_logger_aicam.py
'''
pi@raspberrypi:~ $ sudo pip3 install bluepy
pi@raspberrypi:~ $ git clone https://bokunimo.net/git/maix
pi@raspberrypi:~ $ cd maix
pi@raspberrypi:~/maix $ sudo ./ble_logger_aicam.py

found RN4020 No. 1

to Ambient:
    body {'writeKey': '0123456789abcdef', 'd1': 0, 'd2': 0, 'd3': None, 'd4': None, 'd5': None, 'd6': None, 'd7': None, 'd8': None}
    Done
    チャネルID(ambient_chid)が設定されていません

Device 00:1e:xx:xx:xx:xx (public), RSSI=-64
+----+--------------------------+----------------------------
|type|              description | value
+----+--------------------------+----------------------------
|   1|                    Flags | 04
| 255|             Manufacturer | cd00010c00
+----+--------------------------+----------------------------
    isTargetDev   = RN4020_AICAM
    ID            = 0xcd
    Number        = 1
    Count         = 12
    RSSI          = -59 dB

to Ambient:
    body {'writeKey': '0123456789abcdef', 'd1': 1, 'd2': 12, 'd3': None, 'd4': None, 'd5': None, 'd6': None, 'd7': None, 'd8': None}
    Done
    チャネルID(ambient_chid)が設定されていません
'''

################################################################################
# 引用文献
################################################################################
#・example03_rn4020.py
# Raspberry Pi Pico 内蔵の温度センサの値をBluetoothモジュールRN4020で送信する
# (https://github.com/bokunimowakaru/iot/blob/master/micropython/raspi-pico/)

################################################################################
# 参考文献
################################################################################
#・demo_find_face.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/face_find/)
#・demo_uart_loop.py
# (https://github.com/sipeed/MaixPy_scripts/blob/master/hardware/demo_uart_loop.py
#・GPIO情報、FPIOA情報
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/gpio.html)
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/fpioa.html)
# (https://github.com/sipeed/MaixPy_scripts/blob/master/board/config_maix_dock.py)
#・KPU情報
# (https://wiki.sipeed.com/soft/maixpy/en/api_reference/Maix/kpu.html)
