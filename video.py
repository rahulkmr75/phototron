from PIL import Image
from numpy import *
from pylab import *
import cv2

cap=cv2.VideoCapture(0)
#VideoCapture is an object and the param passed indicates
# the camera
while(True):
	#capture frame by frame
	ret,frame=cap.read()

	#CONVERTING THE IMAGE
	gray =cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

	#displaying the result
	cv2.imshow('frame',gray)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

#releaseing the camera
cap.release()
cv2.destroyAllWindows()

 