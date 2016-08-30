#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import const
import math


def ballp(frame,ball_no):
    #Get the frame

    pub = rospy.Publisher("ballPositions", Point, queue_size=10)
    #Do some transforms
    #rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    b,g,r = cv2.split(frame)
    h,s,v = cv2.split(hsv)


    #Threshold the hue image to get the ball.
    ball = cv2.inRange(hsv,const.LOWball[ball_no],const.UPball[ball_no])

    #erode to reduce noise
    kernel = np.ones((5,5),np.uint8)
    erosion = cv2.erode(ball,kernel,iterations = 1)
    #followed by dilation to increase the size of final ball
    dilation = cv2.dilate(erosion,kernel,iterations = 1)


    # Take the moments to get the centroid
    moments = cv2.moments(dilation)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)

    # Assume no centroid
    centroid = Point(-1,-1,0)

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:

        centroid = Point(centroid_x,centroid_y,0)

    # Put black circle at centroid in image
    #cv2.circle(dilation, ctr, 4, (120,90,100))
    #cv2.rectangle(frame,(centroid_x-12,centroid_y-12),(centroid_x+12,centroid_y+12),(0,0,255),1) 
        
    #show the output
    #cv2.imshow('ball',dilation)
    
    
        
    #store coordinates in centroid
    
    pub.publish(centroid)
    return centroid;
