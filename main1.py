#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Byte
import const
import ballpos as bp
import math
import time
import botpos

dropP=(0,0)


def main():
    centroid=Point(0,0,0)
    rospy.init_node("LinePositions")
    rospy.Subscriber("dropPoint", Point, dropPointLocator)
    
   #defining a reference time
    ref=time.time()

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("output.avi")
    while not rospy.is_shutdown():
    	u+=1
        #Get the frame
        ret, bgr = cap.read()
        if (ret==0):
            break

      	pub4 = rospy.Publisher("table_ends", Quaternion, queue_size=1)
      	#copying data to avoid loss of information
      	bgr2=np.copy(bgr)
      	bgr3=np.copy(bgr)

      	#bot = botpos.findbot(bgr3)

      	#getting the ball position
      	centroid=bp.ballp(bgr2)

        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #b,g,r = cv2.split(bgr)
        h,s,v = cv2.split(hsv)

        #Threshold the hue image to get the ball.
        #line = cv2.inRange(hsv,const.LOW,constUP)


        gline=cv2.inRange(hsv,const.LOWline,const.UPline)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        # ret, gline = cv2.threshold(gline,100,255,cv2.THRESH_BINARY)
        ret, gline = cv2.threshold(gline,-1,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        #cv2.imshow("line",gline)
       
        
        img,temp,hierarchy = cv2.findContours(gline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


        rows,cols=bgr.shape[:2]
        y1=0
        y2=rows-1

        #calculate the end points of the table
        #topmost returns the topmost point in (x,y)
    	if(count2==0):
	        for j in contours:
	        	topmost = tuple(j[j[:,:,1].argmin()][0])
        		bottommost = tuple(j[j[:,:,1].argmax()][0])
        		ends.append([topmost[1],bottommost[1]])
        	count2+=1
        table_ends=Quaternion(ends[0][0],ends[0][1],ends[1][0],ends[1][1])
        pub4.publish(table_ends)

        if cv2.waitKey(32) & 0xFF==ord('q'):
            break

        pub3.publish(st)	

        print "No of frames till now is %d " %u
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def dropPointLocator(point):
	global dropP
	dropP=(int(point.x),int(point.y))

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()
