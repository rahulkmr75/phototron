#!/usr/bin/env python
#HANDLE IMPORTS
import numpy as np
import math
from collections import deque
from sklearn import linear_model
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import time
import cv2
import const

#defining arrays
err=[]
tim=[]
def move(x_drop,y_drop,x_bot,y_bot,ref):
	d=math.sqrt((x_bot-x_drop)**2+(y_drop-y_bot)**2)
	err.append(d)
	if (len(tim)==0):
		tim.append(ref)
	c=time.time()
	tim.append (c)
	integral=0
	'''for i in range(0,len(err)):
		integral=integral+err[i]*(tim[i+1]-tim[i])
	'''
	integral=err[len(err)-1]*(tim[len(tim)-1]-tim[len(tim)-2])
	u=err[len(err)-1]+integral
	print (d,integral)
	
	if (y_drop>y_bot):
		di=1
	elif (y_drop<y_bot):
		di=-1
	else:
		di=0

	return (u,di)

	