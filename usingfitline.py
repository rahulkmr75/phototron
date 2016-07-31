from  PIL import Image
import numpy as np
import cv2
import rospy
from matplotlib import pyplot as plt

img=cv2.imread("image4.jpg")
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
line=cv2.fitLine(gray,CVT_DIST_L1,0,0.01,0.01)
