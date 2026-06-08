import time

import cv2
from pathlib import Path

print("1")
videodevice = cv2.VideoCapture(1)
print("videodevice")

ret, img = videodevice.read()
print("2")
cv2.imwrite("test.jpg", img)
print("3")
time.sleep(1)
print("4")
ret, img = videodevice.read()
print("5")
cv2.imwrite("test2.jpg", img)
print("6")
videodevice.release()
print("7")