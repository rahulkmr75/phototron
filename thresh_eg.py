from PIL import Image
from numpy import *
from pylab import *
import cv2


bgr=cv2.imread("image.jpg")
gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

ret1,th1=cv2.threshold(gray,127,255,cv2.THRESH_BINARY)


ret2,th2=cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


blur = cv2.GaussianBlur(gray,(5,5),0)
ret3,th3 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


cv2.imshow("normal",th1)
cv2.imshow("otsu",th2)
cv2.imshow("gblur",blur)
cv2.imshow("bluredotsu",th3)


cv2.waitKey(0)
cv2.destroyAllwWindows()
