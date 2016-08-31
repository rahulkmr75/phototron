import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#thresh values for the line
LOWtable=np.array([80,100,105])
UPtable=np.array([150,255,255])

#thresh values for the ball
LOWball=np.array(([0,100,150],[0,0,0],[0,0,0],[0,0,0]))
UPball=np.array(([35,255,255],[0,0,0],[0,0,0],[0,0,0]))

#thresh value for the bot
LOWbot=np.array([45,100,75])
UPbot=np.array([150,255,168])

#the thresh value for ball crossing the line 
#and the ball is equidistant between the line
th_ball_on_line=15
th_ball_equidis=15

#thresharea for filtering contours
threshArea = 50

kp=8
