#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
import const
import ballpos as bp
import math
import time
import botpos
import pid_control

dropP=Point(-1,-1,0)
ball_no=0
def main():
    ball=Point(0,0,0)

    #the motor vel publisher
    pub=rospy.Publisher("motor_vel", Point, queue_size=10)

    rospy.init_node("main")
    
    #defining a reference time
    ref=time.time()

    cap = cv2.VideoCapture(1)
    #cap = cv2.VideoCapture("output.avi")
    
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
      	ball=bp.ballp(bgr2,ball_no)

        #gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #b,g,r = cv2.split(bgr)
        #h,s,v = cv2.split(hsv)

        #Threshold the hue image to get the ball.
        #line = cv2.inRange(hsv,const.LOW,constUP)


        table=cv2.inRange(hsv,const.LOWtable,const.UPtable)
        table=cv2.GaussianBlur(table,(15,15),0)
        table=cv2.GaussianBlur(table,(15,15),0)
        table=cv2.GaussianBlur(table,(15,15),0)
        # ret, table = cv2.threshold(table,100,255,cv2.THRESH_BINARY)
        ret, table = cv2.threshold(table,-1,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # Find the coordinates of the drop position
        dropPointLocator(ball,bot)

        bgr=cv2.circle(bgr,(int(dropP.x),int(dropP.y)),10,(255,0,0),-1)
        bgr=cv2.circle(bgr,(ball.x,ball.y),10,(255,0,0),-1)
        bgr=cv2.circle(bgr,(bot.x,bot.y),10,(0,0,255),-1)

        #getting motor velocity and publishing it
        vel_data=pid_control.move(dropP.x,dropP.y,bot.x,bot.y)
        pub.publish(vel_data)

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
