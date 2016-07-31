from PIL import Image
from numpy import *
from pylab import *
import cv2
 
#window to display image
cv2.namedWindow("DIsplay_image")

#reading image
image=cv2.imread("image.jpg") 
h,w=image.shape[:2]

#saving image as anew file
cv2.imwrite("result.jpg",image)

#showing the image
cv2.imshow("DIsplay_image",image)

#exit at closing of window
cv2.waitKey(0)
cv2.destroyAllWindow()
