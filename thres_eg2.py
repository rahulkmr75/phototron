import cv2
import numpy as np
from matplotlib import pyplot as plt
img = cv2.imread('image.jpg',0)
img = cv2.medianBlur(img,5)
ret,th1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
th2 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_MEAN_C,\
	cv2.THRESH_BINARY,11,2)
th3 = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
	cv2.THRESH_BINARY,11,2)

cv2.imshow('thsk',th3)
cv2.imshow('thk',th2)
cv2.imshow('tsk',th1)

cv2.waitKey(0)


