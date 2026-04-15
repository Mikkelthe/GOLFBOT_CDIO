import cv2
from pathlib import Path


videodevice = cv2.VideoCapture(1)


ret, img = videodevice.read()
cv2.imwrite("test.jpg", img)
