from PIL import Image
from numpy import *
from pylab import *
import cv2
from matplotlib import pyplot as plt 


img=cv2.imread('image.jpg')

#changing the colorapce
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

#spliting the image
b,g,r=cv2.split(img)
h,s,v=cv2.split(img)
cv2.imshow("img",img)
cv2.imshow("b",b)

cv2.imshow("hue",h)
ret ,thb=cv2.threshold(b,127,255,cv2.THRESH_BINARY)
ret ,thh=cv2.threshold(h,127,255,cv2.THRESH_BINARY)

cv2.imshow("th",thh)
cv2.imshow("thb",thb)

cv2.waitKey(0)

