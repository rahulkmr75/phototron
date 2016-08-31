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
import pid_control

dropP=Point(-1,-1,0)
ball_no=0
def main():
    centroid=Point(0,0,0)
    rospy.init_node("main")
    #rospy.Subscriber("dropPoint", Point, dropPointLocator)
    
    #defining a reference time
    ref=time.time()

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("output.avi")
    while not rospy.is_shutdown():
        #Get the frame
        ret, bgr = cap.read()
        if (ret==0):
            break


      	#copying data to avoid loss of information
      	bgr2=np.copy(bgr)
      	bgr3=np.copy(bgr)

      	bot = botpos.findbot(bgr3)

      	#getting the ball position
      	centroid=bp.ballp(bgr2,ball_no)

        #gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #b,g,r = cv2.split(bgr)
        #h,s,v = cv2.split(hsv)

        #Threshold the hue image to get the ball.
        #line = cv2.inRange(hsv,const.LOW,constUP)


        gline=cv2.inRange(hsv,const.LOWtable,const.UPtable)
        gline=cv2.GaussianBlur(gline,(15,15),0)
        gline=cv2.GaussianBlur(gline,(15,15),0)
        gline=cv2.GaussianBlur(gline,(15,15),0)
        # ret, gline = cv2.threshold(gline,100,255,cv2.THRESH_BINARY)
        ret, gline = cv2.threshold(gline,-1,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	dropPointLocator(centroid,bot)


        bgr=cv2.circle(bgr,(int(dropP.x),int(dropP.y)),10,(255,0,0),-1)
        bgr=cv2.circle(bgr,(centroid.x,centroid.y),10,(0,255,0),-1)
        bgr=cv2.circle(bgr,(bot.x,bot.y),10,(0,0,255),-1)
        
	pid_control.move(dropP.x,dropP.y,bot.x,bot.y)
	
	cv2.imshow("main",bgr)
        
        if cv2.waitKey(32) & 0xFF==ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def dropPointLocator(point_ball,point_bot):
	global dropP
	dropP=Point(int(point_bot.x),int(point_ball.y),0)

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()
