#!/usr/bin/env python
#HANDLE IMPORTS
import numpy as np
import math
import rospy
from geometry_msgs.msg import Point
import time
import cv2
import const

#defining arrays

tim=[]
def move(x_drop,y_drop,x_bot,y_bot):
	d=math.sqrt((x_bot-x_drop)**2+(y_drop-y_bot)**2)
        u=const.kp*d
	print d

	if(d<15):
		u=0
	elif(d<50 and d>=15):
		u=int(math.tan(0.0523*d)*61)	
	else:
		u=150
	#err.append(d)
	'''if (len(tim)==0):
		tim.append(ref)
	c=time.time()
	tim.append (c)
	integral=0'''

	'''integral=err[len(err)-1]*(tim[len(tim)-1]-tim[len(tim)-2])
	u=err[len(err)-1]+integral
	print (d,integral)
	'''
	if(x_drop==-1 or x_bot==-1 or y_drop==-1 or y_bot==-1):
		u=0
	if (y_drop>y_bot):
		di=1
	elif (y_drop<y_bot):
		di=-1
	else:
		di=0

	return Point(u,di,0)

	
