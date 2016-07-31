from PIL import Image
from numpy import *
from pylab import *
import cv2
 
#window to display image
cv2.namedWindow("DIsplay_color_image")
cv2.namedWindow("Display_grayscale_image")

#reading image
image=cv2.imread("image.jpg") 
h,w=image.shape[:2]
gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

#saving image as anew file
#cv2.imwrite("result.jpg",image)

#showing the image
cv2.imshow("DIsplay_image",image)
cv2.imshow("Display_grayscale_image",gray)

#exit at closing of window
cv2.waitKey(0)
cv2.destroyAllWindow()
