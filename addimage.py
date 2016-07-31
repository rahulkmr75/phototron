import cv2
import numpy as np 
from PIL import Image

img1=cv2.imread("image.jpg")
img2=cv2.imread("image1.jpg")
r1,c1,d=img1.shape
r2,c2,d=img2.shape
if r2>r1:
	r=r2
else:
	r=r1
if c2>c1:
	c=c2
else :
	c=c1

img=np.zeros((r,c,3),dtype=np.uint8)
img[:,:]=img1

#add=cv2.addWeighted(img1,0.6,img2,0.4,0)

cv2.imshow("th",img)


cv2.waitKey(0)
cv2.destroyAllWindows()
flags = [i for i in dir(cv2) if i.startswith('COLOR_')]
print flags