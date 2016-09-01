import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point

#thresh values for the table
LOWtable=np.array([67,18,45]) 
UPtable=np.array([138,101,194])

#thresh values for the ball
LOWball=np.array(([0,100,150],[0,0,0],[0,0,0],[0,0,0]))
UPball=np.array(([35,255,255],[0,0,0],[0,0,0],[0,0,0]))

#thresh value for the bot
#purple colour
#LOWbot=np.array([92,60,120])
#UPbot=np.array([175,130,195])

#green colour
LOWbot=np.array([39,65,122])
UPbot=np.array([75,153,200])


'''#the thresh value for ball crossing the line 
#and the ball is equidistant between the line
th_ball_on_line=15
th_ball_equidis=15'''

#thresharea for filtering contours
threshArea = 50

kp=1
