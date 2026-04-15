import time

import cv2
from pathlib import Path


videodevice = cv2.VideoCapture(1)


ret, img = videodevice.read()
cv2.imwrite("test.jpg", img)
time.sleep(1)
ret, img = videodevice.read()
cv2.imwrite("test2.jpg", img)
videodevice.release()
