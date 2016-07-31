#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point
import math
def main():

    vec1=([0,0],[0,0])
    n1=0
    vec2=([0,0],[0,0])
    n2=0
    # import commands
    # print commands.getoutput("pwd")
    #im = cv2.imread('hsv.png')
    centroid=Point(0,0,0)
    rospy.init_node("LinePositions")

    

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture('test1.webm')
    #cap2 = cv2.VideoCapture('test1.webm')
    while not rospy.is_shutdown():
        #Get the frame
        ret, bgr = cap.read()
        pub = rospy.Publisher("ballPositions", Point, queue_size=10)
        #Do some transforms
        #rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #b,g,r = cv2.split(bgr)
        h,s,v = cv2.split(hsv)

        #Threshold the hue image to get the ball.
        ret, line1 = cv2.threshold(h,100,255,cv2.THRESH_BINARY)
        ret, line2 = cv2.threshold(s,70,255,cv2.THRESH_BINARY)
        ret, line3 = cv2.threshold(v,100,255,cv2.THRESH_BINARY)
        line = cv2.bitwise_and(line1,line2)


        LOW=np.array([80,100,105])
        UP=np.array([150,255,255])
        gline=cv2.inRange(hsv,LOW,UP)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        ret, gline = cv2.threshold(gline,100,255,cv2.THRESH_BINARY)







        cv2.imshow('line2',gline)
        #edges = cv2.Canny(dilation,50,150,apertureSize = 3)
        lines = cv2.HoughLines(gline,1,np.pi/180,300)
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = (int(x0 + 1000*(-b)))
            y1 = (int(y0 + 1000*(a)))
            x2 = (int(x0 - 1000*(-b)))
            y2 = (int(y0 - 1000*(a)))



        #print (rho,theta,x1,y1,x2,y2)
        #kernel = np.ones((5,5),np.float32)/25
        #dst = cv2.filter2D(line,-1,kernel)
        c1,c2=(x1+x2)/2,(y1+y2)/2
        #bgr=cv2.circle(bgr,(0,0),3,(0,0,255),-1)
        
        if(n1<1):   
            vec1=([x1,y1],[x2,y2])
            n1+=1
        else:
            t1=abs(math.sqrt((vec1[0][0]-x1)**2+(vec1[0][1]-y1)**2))
            t2=abs(math.sqrt((vec1[1][0]-x2)**2+(vec1[1][1]-y2)**2))

            if (t1<500 or t2<500):
                a,b,c,d=vec1[0][0],vec1[0][1],vec1[1][0],vec1[1][1]
                vec1[0][0]=(a*n1+x1)/(n1+1)
                vec1[0][1]=(b*n1+x2)/(n1+1)
                vec1[1][0]=(c*n1+x2)/(n1+1)
                vec1[1][1]=(d*n1+y2)/(n1+1)
                n1+=1
            else:
                a,b,c,d=vec2[0][0],vec2[0][1],vec2[1][0],vec2[1][1]
                vec2[0][0]=(a*n2+x1)/(n2+1)
                vec2[0][1]=(b*n2+y1)/(n2+1)
                vec2[1][0]=(c*n2+x2)/(n2+1)
                vec2[1][1]=(d*n2+y2)/(n2+1)
                n2+=1

            #cv2.line(bgr,(vec2[0][0],vec2[0][1]),(vec1[1][0],vec1[1][1]),(100,100,100),2)
        
        if (n1>0 and n2>0):
            cv2.line(bgr,(vec2[0][0],vec2[0][1]),(vec1[1][0],vec1[1][1]),(100,100,100),2)
            cv2.line(bgr,(vec1[0][0],vec1[0][1]),(vec1[1][0],vec1[1][1]),(100,100,100),2)
        #cv2.line(bgr,(x1,y1),(x2,y2),(100,100,100),2)
        


        cv2.imshow('ball',bgr)
        #print (vec1)
        #print (vec2)
        #print(abs(math.sqrt((vec1[0][0]-x1)**2+(vec1[0][1]-y1)**2)))

        #cv2.imshow("line",line)
        #cv2.waitKey(32)
        #publish to topic LinePositions
        if cv2.waitKey(30) & 0xFF==ord('q'):
            break

        rospy.loginfo(centroid)
        pub.publish(centroid)
        




    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()


