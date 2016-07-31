import numpy as np
import cv2

#it creates a black image , to create white image np.ones
#should be used
#np.zeros((x,y),np.uint8)would return a black image but in
#grayscale space 
img=np.zeros((512,512,3),np.uint8)


#draws a blue line of 5 pixel width 
img=cv2.line(img,(0,0),(511,511),(255,0,0),5)
#draws a red line of 5 pixel width 
img=cv2.line(img,(0,511),(511,0),(0,255,0),5)

#drawing a rectangle param-topleft corner,topright corner,
#color,width
img = cv2.rectangle(img,(60,80),(510,200),(0,0,255),3)

#To draw a circle, you need its center coordinates and radius
img = cv2.circle(img,(447,63), 63, (0,0,255), -1)


cv2.imshow('frame',img)

cv2.waitKey(0)
cv2.destroyAllWindows()
