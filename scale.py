from PIL import Image
from numpy import *
from pylab import *
import cv2

bgr=cv2.imread("image.jpg")
gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

bgr2=cv2.imread("image2.jpg")

vec1,vec2=bgr.shape,bgr2.shape
r1,r2,c1,c2=vec1[0],vec2[0],vec1[1],vec2[1]

if r1>r2:
	r=r1
else:
	r=r2

if c1>c2:
	c=c1
else:
	c=c2



bgr=cv2.resize(bgr,(r,c),interpolation=cv2.INTER_CUBIC)
bgr2=cv2.resize(bgr2,(r,c),interpolation=cv2.INTER_CUBIC)

dst = cv2.addWeighted(bgr,0,bgr2,1.0,0)

cv2.imshow("result",dst)

cv2.waitKey(0)
cv2.destroyAllwWindows()