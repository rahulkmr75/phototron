# Load image, convert to grayscale, threshold and find contours
import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point

cap = cv2.VideoCapture('test1.webm')
while(cap.isOpened()):
	ret,img=cap.read()
	
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
	contours,hier = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	cnt = contours[0]
	#image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	# then apply fitline() function
	[vx,vy,x,y] = cv2.fitLine(cnt,cv2.cv.CV_DIST_L2,0,0.01,0.01)

	# Now find two extreme points on the line to draw line
	lefty = int((-x*vy/vx) + y)
	righty = int(((gray.shape[1]-x)*vy/vx)+y)

#Finally draw the line
	cv2.line(img,(gray.shape[1]-1,righty),(0,lefty),255,2)
	cv2.imshow('img',img)
	if cv2.waitKey(30) & 0xFF==ord('q'):
		break
cv2.destroyAllWindows()