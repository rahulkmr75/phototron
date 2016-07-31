from PIL import Image
from numpy import *
from pylab import *
import cv2

cap=cv2.VideoCapture(0)

#defining the codec and creating VideoWriter object

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out=cv2.VideoWriter('output.avi',fourcc,20.0,(640,480))
gout=cv2.VideoWriter('goutput.avi',fourcc,20.0,(640,480))
iout=cv2.VideoWriter('ioutput.avi',fourcc,20.0,(640,480))

while(cap.isOpened()):
	ret,frame=cap.read()
	if ret==True:
		#here the param 0 makes the frame inverted and 
		#non zero has no effect
		iframe=cv2.flip(frame,0)
		gframe=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

		#writing the file
		out.write(frame)
		gout.write(gframe)
		iout.write(iframe)

		cv2.imshow('frame',frame)
		cv2.imshow('iframe',iframe)
		cv2.imshow('gframe',gframe)
		if cv2.waitKey(1) & 0xFF==ord('q'):
			break

	else:
		break

#releasing everything

cap.release()
out.release()
cv2.destroyAllWindow()

