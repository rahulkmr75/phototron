import numpy as np
import math
from collections import deque
from sklearn import linear_model
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import time
import cv2
from matplotlib import pyplot as plt
import pid_control as pid


ref= time.time()
for i in range(1,200):

	u,di=pid.move(10,389,10,math.sqrt(i)*27,ref)
	print (u,di)
	time.sleep(0.03)



