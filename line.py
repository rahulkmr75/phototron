#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point

def main():
    # import commands
    # print commands.getoutput("pwd")
    #im = cv2.imread('hsv.png')
    centroid=Point(0,0,0)
    rospy.init_node("LinePositions")

    

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture('test2.webm')
    #cap2 = cv2.VideoCapture('test2.webm')
    
    while not rospy.is_shutdown():
        n=0
        x1avg=0
        x2avg=0
        y1avg=0
        y2avg=0
        x3avg=0
        x4avg=0
        y3avg=0
        y4avg=0
        #Get the frame
        ret, bgr = cap.read()
        bgr = cv2.GaussianBlur(bgr,(5,5),0)
        src2=cv2.imread("Mask.jpg")
        b,g,r = cv2.split(src2)
        ret, src2 = cv2.threshold(b,100,255,cv2.THRESH_BINARY)
        ret, src3 = cv2.threshold(b,100,255,cv2.THRESH_BINARY_INV)
        
        pub1 = rospy.Publisher("LinePositions1", Point, queue_size=10)
        pub2 = rospy.Publisher("LinePositions2", Point, queue_size=10)
        #Do some transforms
        #rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #b,g,r = cv2.split(bgr)
        h,s,v = cv2.split(hsv)
        #cv2.imshow('crap',src2)
        #Threshold the hue image to get the ball.
        ret, line_a = cv2.threshold(h,100,255,cv2.THRESH_BINARY)
        ret, line_b = cv2.threshold(s,70,255,cv2.THRESH_BINARY)
        ret, line_c = cv2.threshold(v,100,255,cv2.THRESH_BINARY)
        line_both = cv2.bitwise_and(line_a,line_b)
        line1 = cv2.bitwise_and(line_both,src2)
        line2 = cv2.bitwise_and(line_both,src3)
        #cv2.imshow('line1',line1)
        #cv2.imshow('line2',line2)
        #edges = cv2.Canny(dilation,50,150,apertureSize = 3)
        lines = cv2.HoughLines(line1,1,np.pi/180,300)
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
        lines1 = cv2.HoughLines(line2,1,np.pi/180,300)
        for rho,theta in lines1[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x3 = int(x0 + 1000*(-b))
            y3 = int(y0 + 1000*(a))
            x4 = int(x0 - 1000*(-b))
            y4 = int(y0 - 1000*(a))
            
        x1avg+=x1
        x2avg+=x2
        y1avg+=y1
        y2avg+=y2
        x3avg+=x3
        x4avg+=x4
        y3avg+=y3
        y4avg+=y4
        n+=1
        cv2.line(bgr,(x1avg/n,y1avg/n),(x2avg/n,y2avg/n),(0,0,255),2)
        cv2.line(bgr,(x3avg/n,y3avg/n),(x4avg/n,y4avg/n),(0,0,255),2)
        

        if (cv2.waitKey(32) & 0xFF==ord('q')):
            break
        #publish to topic LinePositions
        if ((x1+x2)>0 and (x3+x4)>0 and (y1+y2)>0 and (y3+y4)>0):
            line_one = Point((x1+x2)/2,(y1+y2)/2,0)
            line_two = Point((x3+x4)/2,(y3+y4)/2,0)
            cv2.rectangle(bgr,((x1+x2)/2-4,296),((x1+x2)/2+4,304),(0,255,0),1) 
            cv2.rectangle(bgr,((x3+x4)/2-4,296),((x3+x4)/2+4,304),(0,255,0),1)
            rospy.loginfo(line_one)
            pub1.publish(line_one)
            rospy.loginfo(line_two)
            pub2.publish(line_two)
        cv2.imshow('ball',bgr)


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()

