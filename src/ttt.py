import pyautogui
from PIL import ImageGrab
import time


# 마우스 좌표 출력
while True:
    screen = ImageGrab.grab() # 화면 캡쳐
    color = screen.getpixel(pyautogui.position(x=1103, y=906))
    print(color)#현재의 마우스 위치의 색상 출력.
    # print( "Current Mouse Position : ", pyautogui.position() ) 
    if color == (51, 32, 213):
        print('blue!')
        pyautogui.moveTo(x=1103, y=906)
        pyautogui.click()
    time.sleep(3)