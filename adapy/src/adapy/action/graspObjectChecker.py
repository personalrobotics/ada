#!/usr/bin/env python
# Developed by Lerrel Pinto, RI, CMU, lerrelp@andrew.cmu.edu
# Modified by Shen Li
import rospy
import numpy as np
import scipy
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import sys
from scipy import signal
class DetectObjectGrasp:
	def __init__(self):

		self.finger1RawData = np.array([]) 
		self.finger2RawData = np.array([]) 
		self.finger1FilteredData = np.array([]) 
		self.finger2FilteredData = np.array([]) 
		
		self.fingerBufferMaxSize = 300
		# Butterworth digital and analog filter design.
		self.b, self.a = scipy.signal.butter(4, 0.5, 'low', analog=False)
		self.objPub = rospy.Publisher('object_grasp_flag',Bool, queue_size = 1)
		rospy.Subscriber('/joint_states', JointState, self.fingerEffortCallback)
		'''
		header: 
		  seq: 317195
		  stamp: 
		    secs: 1467860315
		    nsecs: 587810400
		  frame_id: ''
		name: ['mico_joint_1', 'mico_joint_2', 'mico_joint_3', 'mico_joint_4', 'mico_joint_5', 'mico_joint_6', 'mico_joint_finger_1', 'mico_joint_finger_2']
		position: [2.5073066446592396, -2.43672342155027, -2.155995083671164, 2.410934368239351, -1.0412477830052518, 0.09281968610474971, 0.1998888888888889, 0.20066666666666666]
		velocity: [6.910177216874543e-22, -2.201154480504752e-46, 2.201154480504752e-46, 2.201154480504752e-46, 2.201154480504752e-46, 2.201154480504752e-46, 0.0, 0.0]
		effort: [0.004984275028394676, 0.19163254992661372, -0.0645005465344718, -0.014301286365536696, -0.010938404666174358, 0.0002240237812674779, 0.0, 0.0]
		'''
		rospy.spin()
	
	def fingerEffortCallback(self,data):
		finger1Val = data.effort[-2]
		finger2Val = data.effort[-1]
		# push new data into the buffer list
		if self.finger1RawData.size < self.fingerBufferMaxSize:
			self.finger1RawData = np.append(self.finger1RawData,np.array([finger1Val]))
		else:
			# if the buffer is full, we remove the earliest data and append the new data point to the end
			self.finger1RawData = np.append(self.finger1RawData[1:],np.array([finger1Val]))
		
		if self.finger2RawData.size < self.fingerBufferMaxSize:
			self.finger2RawData = np.append(self.finger2RawData,np.array([finger2Val]))
		else:
			self.finger2RawData = np.append(self.finger2RawData[1:],np.array([finger2Val]))

		self.finger1FilteredData = scipy.signal.lfilter(self.b, self.a, self.finger1RawData)
		self.finger2FilteredData = scipy.signal.lfilter(self.b, self.a, self.finger2RawData)
		self.fingerMax = self.finger1FilteredData.max() + self.finger2FilteredData.max()
		import IPython;IPython.embed()
		if self.fingerMax > 0.0001:
			print "True",self.fingerMax
			self.objPub.publish(Bool(True))	
		else:
			print "False",self.fingerMax
			self.objPub.publish(Bool(False))	
			

def main():
	print "Beginning node"
	rospy.init_node('GraspVerify')
	DetectObjectGrasp()
	sys.exit(0)

if __name__ == "__main__":
	main()

