#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
def main():

    vec1=[[0,0],[0,0]]
    n1=0
    vec2=[[0,0],[0,0]]
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
        pub1 = rospy.Publisher('Line1Positions', Quaternion, queue_size=10)
        pub2 = rospy.Publisher("Line2Positions", Quaternion, queue_size=10)        
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
        # ret, gline = cv2.threshold(gline,100,255,cv2.THRESH_BINARY)
        ret, gline = cv2.threshold(gline,-1,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


        







        cv2.imshow("line",gline)
        #edges = cv2.Canny(dilation,50,150,apertureSize = 3)
        '''lines = cv2.HoughLines(gline,1,np.pi/180,300)
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = (int(x0 + 1000*(-b)))
            y1 = (int(y0 + 1000*(a)))
            x2 = (int(x0 - 1000*(-b)))
            y2 = (int(y0 - 1000*(a)))
            cv2.line(bgr,(x1,y1),(x2,y2),(0,255,0),2)'''
        	




        #kernel = np.ones((5,5),np.float32)/25
        #dst = cv2.filter2D(line,-1,kernel)

        '''if(n1<1):   
            vec1=([x1,y1],[x2,y2])
            n1+=1
        if(len(vec1)==1 and abs(vec1[1][1]-y2)>500):
            vec2=(  [(x1+vec2[0][0]*n2)/(n2+1),(y1+vec2[0][1]*n2)/(n2+1)]  ,  [(x2+vec2[1][0]*n2)/(n2+1),(y2+vec2[1][1]*n2)/(n2+1)])
            n2+=1
        if (n1>0 and abs(vec1[1][1]-y2)<500):
            vec1=(  [(x1+vec1[0][0]*n1)/(n1+1),(y1+vec1[0][1]*n1)/(n1+1)]  ,  [(x2+vec1[1][0]*n1)/(n1+1),(y2+vec1[1][1]*n1)/(n1+1)])
            n1+=1'''


            #cv2.line(bgr,(vec2[0][0],vec2[0][1]),(vec1[1][0],vec1[1][1]),(100,100,100),2)
        
        threshArea = 500
        img,temp,hierarchy = cv2.findContours(gline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #bgr = cv2.drawContours(bgr,temp, -1, (0,255,0), 3)

        #contours = temp[0]
        #hierarchy = temp[1]
        contours2 = []
        for i in temp:
         	if cv2.contourArea(i)>threshArea:
         		contours2.append(i)
        contours = contours2

        #print "NO OF CONTOURS ",len(contours)

        #vec1=cv2.fitLine(contours[0],cv2.DIST_L2,0,0.01,0.01)
        #vec2=cv2.fitLine(contours[1],cv2.DIST_L2,0,0.01,0.01)
        #cv2.line(bgr,(vec2[0][0],vec2[0][1]),(vec1[1][0],vec1[1][1]),(100,100,100),2)
        #cv2.line(bgr,(vec1[0][0],vec1[0][1]),(vec1[1][0],vec1[1][1]),(100,100,100),2)
        #print vec1,vec2
        rows,cols=bgr.shape[:2]
        [vx,vy,x,y] = cv2.fitLine(contours[0], cv2.DIST_L2,0,0.01,0.01)

        y1=0
        y2=rows-1
        x1=int(vx/vy*(y1-y)+x)
        x2=int(vx/vy*(y2-y)+x)
        #vec = cv2.fitLine(contours[0], cv2.DIST_L2,0,0.01,0.01)
        #lefty = int((-x*vy/vx) + y)
        #righty = int(((cols-x)*vy/vx)+y)
        bgr = cv2.line(bgr,(x1,y1),(x2,y2),(0,255,0),2)
        #bgr=cv2.circle(bgr,(y1,x1),4,(255,0,0),3,-1)

        [vx,vy,x,y] = cv2.fitLine(contours[1], cv2.DIST_L2,0,0.01,0.01)
        x3=int(vx/vy*(y1-y)+x)
        x4=int(vx/vy*(y2-y)+x)
        bgr = cv2.line(bgr,(x3,y1),(x4,y2),(0,255,0),2)
       
        
        cv2.imshow('ball',bgr)
        #print (vx,vy,x,y)
        #print x1,y1,x2,y2,x3,x4
        #cv2.imshow("line",line)
        #cv2.waitKey(32)
        #publish to topic LinePositions
        if cv2.waitKey(30) & 0xFF==ord('q'):
            break
        pub1.publish(Quaternion(x1,y1,x2,y2))
        pub2.publish(Quaternion(x3,y1,x4,y2))
        




    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()


