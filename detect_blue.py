import numpy as np 
import cv2
from matplotlib import pyplot as plt 
 
cap=cv2.VideoCapture(0);

while (cap.isOpened()):
	ret,bgr=cap.read()
	gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
	hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
	

	b=bgr[:,:,1]
	# define range of ______ color in HSV
	lower_blue = np.array([50,100,100])
	upper_blue = np.array([70,255,255])
	
	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_blue, upper_blue)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(bgr,bgr, mask= mask)
	res2=cv2.bitwise_and(b,mask)
	cv2.imshow('frame',bgr)
	cv2.imshow('mask',mask)
	cv2.imshow('res',res)
	cv2.imshow('res2',res2)

	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break