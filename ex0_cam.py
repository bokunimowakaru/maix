###############################################################################
# Example 0 カメラの画像を表示する for Sipeed Maix Dock
#
#                        Copyright (c) 2021 Wataru KUNINO https://bokunimo.net/
###############################################################################

import sensor, lcd                                  # カメラsensor,液晶lcdの組み込み
lcd.init()                                          # LCDの初期化
sensor.reset()                                      # カメラの初期化
sensor.set_pixformat(sensor.RGB565)                 # カラー設定（モノクロ時GRAYSCALE)
sensor.set_framesize(sensor.QVGA)                   # 解像度設定(QVGA:320x240)

while(True):                                        # 永久ループ
    img = sensor.snapshot()                         # 写真を撮影しオブジェクトimgに代入
    img.draw_string(5,180,"Hello, World!",scale=3)  # 文字列「Hello～」をimgに追記
    img.draw_string(15,210,"bokunimo.net",scale=3)  # 文字列「bokunimo～」っを追記
    lcd.display(img)                                # 以上の結果をLCDに表示

###############################################################################
# (参考文献)
# ・demo_fps_display.py
#   (https://github.com/sipeed/MaixPy_scripts/blob/master/machine_vision/)
# ・helloworld.py (MaixPy IDE 0.2.5)
#   (https://dl.sipeed.com/shareURL/MAIX/MaixPy/ide/)

'''
# Hello World Example
#
# Welcome to the MaixPy IDE!
# 1. Conenct board to computer
# 2. Select board at the top of MaixPy IDE: `tools->Select Board`
# 3. Click the connect buttion below to connect board
# 4. Click on the green run arrow button below to run the script!

import sensor, image, time, lcd

lcd.init(freq=15000000)
sensor.reset()                      # Reset and initialize the sensor. It will
                                    # run automatically, call sensor.run(0) to stop
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.

while(True):
    clock.tick()                    # Update the FPS clock.
    img = sensor.snapshot()         # Take a picture and return the image.
    lcd.display(img)                # Display on LCD
    print(clock.fps())              # Note: MaixPy's Cam runs about half as fast when connected
                                    # to the IDE. The FPS should increase once disconnected.
'''
