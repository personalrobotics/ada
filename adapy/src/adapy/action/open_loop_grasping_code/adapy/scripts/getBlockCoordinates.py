#! /usr/bin/env python

import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
import sys
from cv_bridge import CvBridge, CvBridgeError
from IPython import embed
import copy

def callbackDepth(data):
	try:

		## First import data into array
		N = np.fromstring(data.data,dtype = np.float32)
		depth_array = np.reshape(N,(480,640))
		
		## Now correct for depth holes by assigning to the maximum value
		depth_array[np.isnan(depth_array)] = 0
		depth_array[depth_array==0] = depth_array.max()

		## Ideally now the blocks are almost black in the depth_array 
		depth_image = copy.deepcopy(depth_array)
		cv2.normalize(depth_image,depth_image, 0, 1, cv2.NORM_MINMAX)
		depth_detect = copy.deepcopy(depth_image)*0
		depth_detect[depth_image<0.2] = 1
		cv2.imshow("depth_image",depth_image)
		cv2.imshow("depth_detect",depth_detect)
		cv2.waitKey(1)

		## Blob detection
		cv2.normalize(depth_array,depth_array, 0, 1, cv2.NORM_MINMAX)
		blob_disp_image = cv2.cvtColor(depth_array,cv2.COLOR_GRAY2RGB)
		blob_image = cv2.cvtColor(depth_detect,cv2.COLOR_GRAY2RGB)
		blob_image = blob_image.astype(np.uint8)
		blob_image_bw = cv2.cvtColor(blob_image,cv2.COLOR_RGB2GRAY)

		blur_img = cv2.blur(blob_image_bw,(5,5))
		contours,hierarchy = cv2.findContours(blur_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		max_area_threshold = 100
		for cnt in contours:
			area = cv2.contourArea(cnt)
			if area > max_area_threshold:
				M = cv2.moments(cnt)
				cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
				x,y,w,h = cv2.boundingRect(cnt)
				cv2.rectangle(blob_disp_image,(x,y),(x+w,y+h),(0,255,0),2)
				cv2.circle(blob_disp_image,(cx,cy),10,(0,0,255),-1)
		cv2.imshow("depth_image_detect",blob_disp_image)
		cv2.waitKey(1)
		#cv2.imwrite('capture.png',depth_array)
	except CvBridgeError, e:
		print e
	
rospy.init_node('test_depth')
rospy.Subscriber('camera/depth/image',Image,callbackDepth)
rospy.spin()

