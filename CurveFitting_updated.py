#!/usr/bin/env python
#HANDLE IMPORTS
import numpy as np
import math
from collections import deque
from sklearn import linear_model
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

#DEFINE CURVE PARAMETERS GLOBALLY
params = np.empty(3)
#DEFINE DROP POINT Y COORDINATE GLOBALLY
top_end=0
bottom_end=0
dropX = 0
dropY = 0
n = 0 #No of point used to take average
#DEFINE THE QUEUE IN WHICH BALL POSITIONS ARE STORED
ballPositions = deque(maxlen=20)
#define the list used to calc avg value of the drop point
dropY_list = deque(maxlen=3)

#ROS CALLBACK FUNCTION FOR BALL POSITION
def callbackBallPosition(point):
    #INSERT THE NEWEST BALL POSITION INTO A Queue.Queue OR collections.deque
    ballPositions.append(point);

    #CREATE A TRAINING DATASET FROM DEQUE
    temp = list(iter(ballPositions))
    tempX = [p.x for p in temp]
    tempY = [p.y for p in temp]
    # trainX = tempX
    # trainY = [[y, y*y] for y in tempY]
    trainX = [[x, x*x] for x in tempX]
    trainY = tempY
    
    #DO LINEAR REGRESSION ON THE QUEUE TO GET THE CURVE SHAPE
    #ITS SAFE TO ASSUME A SECOND DEGREE EQUATION FOR THE CURVE
    regr = linear_model.LinearRegression()
    # regr.fit(trainY, trainX)
    regr.fit(trainX, trainY)
    
    #UPDATE GLOBAL CURVE PARAMETERS
    global params
    params = regr.coef_
    params = np.fliplr([params])[0]
    params = np.append(params, regr.intercept_)
    rospy.loginfo(params)

#ROS CALLBACK FUNCTION FOR DROP POINT Y COORIDINATE
def callbackDropPointY(point):
#     #FIND THE MODE OR MEAN OF DROP POINT  Y COORDINATE
#     #UPDATE GLOBAL DROP POINT Y COORDINATE
    global dropY
    global n
    global dropY_list
    sum1 = 0
    if (point>=0 and point<=720):
    	dropY_list.append(point)
    temp= list(iter(dropY_list))
    for p in temp:
    	sum1+=p
    dropY=sum1/3
    
    #if the drop point goes outside the table, fix the drop point to the nearest end
    if(dropY<=top_end):
    	dropY = top_end
    elif(dropY>=bottom_end):
    	dropY=bottom_end
    return dropY

#PUBLISHER FUNCTION
    #CALCULATE DROP POINT Y COORDINATE FROM GLOBAL VARIABLES
    #PUBLISH THE DROP POINT Y COORDINATE
def publishDropPoint(line2pos):

    dropY_new=0.0
    dropY_temp=0.0
    pub = rospy.Publisher("dropPoint", Point, queue_size=10)
    
    x1=line2pos.x
    y1=line2pos.y
    x2=line2pos.z
    y2=line2pos.w
    
    #ax^2+bx+c is the predicted curve
    a  = params[0]
    b  = params[1]
    c  = params[2]

    #ax+by+c=0
    
    if ((x2-x1)==0):
    	dropX=x1
    	dropY_temp=a*x1*x1+b*x1+c
    	if (dropY_temp>0 and dropY_temp<720):
    		dropY_new=dropY_temp


    #m  = (float) (y2-y1)/(x2-x1)
    #c0 = y1-x1*(m) 

    else:
    	c0=y1-x1*(y1-y2)/(x1-x2)
    	m=(y2-y1)/(x2-x1)
    	print "c0 =  %d and m = %d" % (c0,m)
    	dropX1 = (-b + (m) + math.sqrt((b - m)**2 - 4*a*(c - c0)))/2*a
    	dropX2 = (-b + (m) - math.sqrt((b - m)**2 - 4*a*(c - c0)))/2*a
    	if (dropX1>=0 and dropX1<=1280):
        	dropX=dropX1
    	elif (dropX2>=0 and dropX2<=1280):
        	dropX=dropX2
        dropY_new = m*dropX + c0
    
    #calculate dropY
    
    dropY_new = callbackDropPointY(dropY_new)
    rospy.loginfo(dropY_new)
    pub.publish(Point(dropX,dropY_new,0))
    
def endPoint(ends):
	global top_end
	global bottom_end
	top_end = ends.x
	bottom_end = ends.y

#MAIN FUNCTION OF THIS MODULE
    #SUBSCRIBE TO BALL POSITION
    #SUBSCRIBE TO DROP POINT Y COORDINATE
    #RUN THE PUBLISHER FUNCTION
    #KEEP THE MODULE RUNNING
def main():
    rospy.init_node("CurveFitting")
    
    rospy.Subscriber("ballPositions", Point, callbackBallPosition)
    rospy.Subscriber("Line1Positions",Quaternion, publishDropPoint)
    rospy.Subscriber("table_ends",Quaternion, endPoint)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()


