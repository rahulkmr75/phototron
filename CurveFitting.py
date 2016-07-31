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
dropX = 0
dropY = 0
n = 3 #No of point used to take average
#DEFINE THE QUEUE IN WHICH BALL POSITIONS ARE STORED
ballPositions = deque(maxlen=100)


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
    params = regr.coef_
    params = np.fliplr([params])[0]
    params = np.append(params, regr.intercept_)

    rospy.loginfo(params)

#ROS CALLBACK FUNCTION FOR DROP POINT Y COORIDINATE
def callbackDropPointX(point):
#     #FIND THE MODE OR MEAN OF DROP POINT  X COORDINATE
#     #UPDATE GLOBAL DROP POINT X COORDINATE
     dropX = dropX*n + point.x
     n = n+1
     dropX = dropX/n;
     print dropX

#PUBLISHER FUNCTION
    #CALCULATE DROP POINT X COORDINATE FROM GLOBAL VARIABLES
    #PUBLISH THE DROP POINT X COORDINATE
def publishDropPoint(line2pos):
    pub = rospy.Publisher("dropY", Point, queue_size=10)
    x1=line1pos[0]
    y1=line1pos[1]
    x2=line1pos[2]
    y2=line1pos[3]
    c0=y1-x1*((y2-y1)/(x2-x1))
    a=params[0]
    b=params[1]
    c=params[2]
    dropY1 =(-b+((y2-y1)/(x2-x1))+math.sqrt((b-((y2-y1)/(x2-x1)))**2-4*a*(c-c0)))/2*a
    dropY2 =(-b+((y2-y1)/(x2-x1))-math.sqrt((b-((y2-y1)/(x2-x1)))**2-4*a*(c-c0)))/2*a
    if (dropY1>0 and dropY1<720):
        dropY=dropY1
    elif (dropY2>0 and dropY2<720):
        dropY=dropY2
    rospy.loginfo(dropY)
    pub.publish(Point(0,dropY,0))

def turnOnLED(line1pos):
    x1=line1pos[0]
    y1=line1pos[1]
    x2=line1pos[2]
    y2=line1pos[3]
    a=params[0]
    b=params[1]
    c=params[2]
#MAIN FUNCTION OF THIS MODULE
    #SUBSCRIBE TO BALL POSITION
    #SUBSCRIBE TO DROP POINT Y COORDINATE
    #RUN THE PUBLISHER FUNCTION
    #KEEP THE MODULE RUNNING
def main():
    rospy.init_node("CurveFitting")

    rospy.Subscriber("ballPositions", Point, callbackBallPosition)
    rospy.Subscriber("Line1Positions",Quaternion, publishDropPoint)
    rospy.Subscriber("Line2Positions",Quaternion, turnOnLED)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()


