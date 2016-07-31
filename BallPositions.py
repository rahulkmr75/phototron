#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point

def main():
    #initialise centroid
    centroid=Point(0,0,0)
    rospy.init_node("BallPositions")

    

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture('test2.webm')

    while not rospy.is_shutdown():
        #Get the frame
        ret, bgr = cap.read()
        pub = rospy.Publisher("ballPositions", Point, queue_size=10)
        #Do some transforms
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        b,g,r = cv2.split(bgr)
        h,s,v = cv2.split(hsv)



        #Threshold the hue image to get the ball.
        ret, ball1 = cv2.threshold(h,30,255,cv2.THRESH_BINARY_INV)
        ret, ball2 = cv2.threshold(s,100,255,cv2.THRESH_BINARY)
        ret, ball3 = cv2.threshold(v,150,255,cv2.THRESH_BINARY)
        ball = cv2.bitwise_and(ball1,ball2)

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
        ctr = (-1,-1)

        # Use centroid if it exists
        if centroid_x != None and centroid_y != None:

            ctr = (centroid_x, centroid_y)

        # Put black circle at centroid in image
        cv2.circle(dilation, ctr, 4, (120,90,100))
        cv2.rectangle(bgr,(centroid_x-12,centroid_y-12),(centroid_x+12,centroid_y+12),(0,0,255),1) 
        
        #show the output
        cv2.imshow('ball',dilation)
        cv2.imshow('bgr',bgr)
        cv2.waitKey(300)
        
        #store coordinates in centroid
        centroid = Point(centroid_x,centroid_y,0)

        #publish the centroid
        rospy.loginfo(centroid)
        pub.publish(centroid)


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()

