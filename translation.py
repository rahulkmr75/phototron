from PIL import Image
from numpy import *
from pylab import *
import cv2

bgr=cv2.imread("image.jpg")
gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

bgr2=cv2.imread("image2.jpg")

rows,cols,channel=bgr.shape
#shifting the image
M=np.float32([[1,0,20],[0,1,30]])
shifted=cv2.warpAffine(bgr,M,(cols,rows))
cv2.imshow("image",bgr)
cv2.imshow("shifted",shifted)

cv2.waitKey(0)
cv2.destroyAllWindows()