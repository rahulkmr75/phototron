#!/usr/bin/python

# Standard imports
import cv2
import numpy as np;
import rospy
from matplotlib import pyplot as plt
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Byte
import const
import ballpos as bp
import math
import time
import botpos

dropP=(0,0)


def main():
    centroid=Point(0,0,0)
    rospy.init_node("LinePositions")
    rospy.Subscriber("dropPoint", Point, dropPointLocator)
    
    # for the t2-t1 values of the ball and the line
    err1=[]
    err2=[]
    err3=[]
    u=0

    vec1=[[0,0],[0,0],[0,0]]
    vec2=[[0,0],[0,0],[0,0]]
    n1=0
    n2=0

    #variables for the state of led and the state changing state
    st=0
    stcst=1
    count=0
    count2=0
    ends=[]

    #defining a reference time
    ref=time.time()

    #cap = cv2.VideoCapture(0)
    cap = cv2.VideoCapture("output.avi")
    while not rospy.is_shutdown():
    	u+=1
        #Get the frame
        ret, bgr = cap.read()
        if (ret==0):
            break
        pub1 = rospy.Publisher('Line1Positions', Quaternion, queue_size=10)
        pub2 = rospy.Publisher("Line2Positions", Quaternion, queue_size=10)        
      	pub3 = rospy.Publisher("LED", Byte, queue_size=10)
      	pub4 = rospy.Publisher("table_ends", Quaternion, queue_size=1)
      	#copying data to avoid loss of information
      	bgr2=np.copy(bgr)
      	bgr3=np.copy(bgr)

      	#bot = botpos.findbot(bgr3)

      	#getting the ball position
      	centroid=bp.ballp(bgr2)

        gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        #b,g,r = cv2.split(bgr)
        h,s,v = cv2.split(hsv)

        #Threshold the hue image to get the ball.
        #line = cv2.inRange(hsv,const.LOW,constUP)


        gline=cv2.inRange(hsv,const.LOWline,const.UPline)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        gline=cv2.GaussianBlur(gline,(5,5),0)
        # ret, gline = cv2.threshold(gline,100,255,cv2.THRESH_BINARY)
        ret, gline = cv2.threshold(gline,-1,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        #cv2.imshow("line",gline)
       
        
        img,temp,hierarchy = cv2.findContours(gline, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        contours2 = []
        for i in temp:
         	if cv2.contourArea(i)>const.threshArea:
         		contours2.append(i)
        contours = contours2


        rows,cols=bgr.shape[:2]
        y1=0
        y2=rows-1

        #calculate the end points of the table
        #topmost returns the topmost point in (x,y)
    	if(count2==0):
	        for j in contours:
	        	topmost = tuple(j[j[:,:,1].argmin()][0])
        		bottommost = tuple(j[j[:,:,1].argmax()][0])
        		ends.append([topmost[1],bottommost[1]])
        	count2+=1
        table_ends=Quaternion(ends[0][0],ends[0][1],ends[1][0],ends[1][1])
        pub4.publish(table_ends)
        


        for i in range(0,2):
        	if (len(contours)<i):
        		break
        	[vx,vy,x,y] = cv2.fitLine(contours[i], cv2.DIST_L2,0,0.01,0.01)
        	#the x coordinate corresponding to y=0
        	x1=int(vx/vy*(y1-y)+x)
        	#the x coordinate corresponding to y=rows-1
        	x2=int(vx/vy*(y2-y)+x)
        	#whatever comes first append it to vec1
        	
        	if (n1==0):
        		vec1=[[x1,y1],[x2,y2],[vx,vy]]
        		n1=n1+1

        	if (len(vec1)>0 and abs(vec1[0][0]-x1)>80):
        		vec2=([(vec2[0][0]*n2+x1)/(n2+1),y1],[(vec2[1][0]*n2+x1)/(n2+1),y2],[(vec2[2][0]*n2+vx)/(n2+1),(vec2[2][1]*n2)/(n2+1)])
        		n2+=1

        	if (len(vec1)>0 and (abs(vec1[0][0]-x1))<80):
        		vec1=([(vec1[0][0]*n1+x1)/(n1+1),y1],[(vec1[1][0]*n1+x1)/(n1+1),rows-1],[(vec1[2][0]*n1+vx)/(n1+1),(vec1[2][1]*n1)/(n1+1)])
       			n1+=1

       

       	d1=bp.bonline2(centroid,vec1[1][0],vec1[1][1],vec1[0][0],vec1[0][1])
       	d2=bp.bonline2(centroid,vec2[1][0],vec2[1][1],vec2[0][0],vec2[0][1])

        #drawing the two lines obtained
        bgr=cv2.line(bgr,(vec2[1][0],vec2[1][1]),(vec2[0][0],vec2[0][1]),(0,0,255),2)
        bgr=cv2.line(bgr,(vec1[1][0],vec1[1][1]),(vec1[0][0],vec1[0][1]),(0,0,255),2)

       	err1.append(d1)
       	err2.append(d2)
       	
     	err3.append(abs(d1-d2))

        #the led thing
        if (count<2):
        	if (stcst==1):
        		if(d1<const.th_ball_on_line or d2<const.th_ball_on_line):
        			if (st==1):
        				st=0
        			else:
        				st=1
        			count+=1
        			stcst=0
        	if (abs(d1-d2)<const.th_ball_equidis):
        		stcst=1
        else:
        	count=0
        

        #turning the led on
        if (st==1):
        	bgr=cv2.circle(bgr,(20,20),10,(255,0,0),-1)
       
        bgr=cv2.circle(bgr,dropP,10,(255,0,0),-1)
        cv2.imshow('ball',bgr)
        
        if cv2.waitKey(32) & 0xFF==ord('q'):
            break
        pub1.publish(Quaternion(vec1[0][0],vec1[0][1],vec1[1][0],vec1[1][1]))
        #pub2.publish(Quaternion(x3,y1,x4,y2))
        pub3.publish(st)	
        #getting the ball position
        print "No of frames till now is %d " %u
    '''plt.plot(err1)
    plt.plot(err2)
    plt.plot(err3)
    plt.show()
    cv2.waitKey(0)'''

        




    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def dropPointLocator(point):
	global dropP
	dropP=(int(point.x),int(point.y))



#IF __main__ RUN THE MAIN FUNCTION. THIS IS THE MAIN THREAD
if __name__ == '__main__':
    main()



