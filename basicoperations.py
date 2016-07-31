import cv2
import numpy as np

#reading the image
img=cv2.imread("image.jpg")

#this prints the red value of each pixel in the image
size=img.shape
for i in range(0,size[0]):
	for j in range(0,size[1]):
		print("the red value is", img[i,j,2],)	
		#or print ("the green value is ",img.item[i,j,1])


		#img[i,j,2] i is for row j is for column the third
		#parameter is for channel 0- blue ;1-green ;2-red
		#img[i,j] returns all the three values for the color



