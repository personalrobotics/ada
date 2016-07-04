#!/usr/bin/env python
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
		self.b, self.a = scipy.signal.butter(4, 0.5, 'low', analog=False)
		self.objPub = rospy.Publisher('object_grasp_flag',Bool, queue_size = 1)
		rospy.Subscriber('/joint_states', JointState, self.fingerEffortCallback)
		rospy.spin()
	
	def fingerEffortCallback(self,data):
		finger1Val = data.effort[0]
		finger2Val = data.effort[1]
		if self.finger1RawData.size < self.fingerBufferMaxSize:
			self.finger1RawData = np.append(self.finger1RawData,np.array([finger1Val]))
		else:
			self.finger1RawData = np.append(self.finger1RawData[1:],np.array([finger1Val]))
		
		if self.finger2RawData.size < self.fingerBufferMaxSize:
			self.finger2RawData = np.append(self.finger2RawData,np.array([finger2Val]))
		else:
			self.finger2RawData = np.append(self.finger2RawData[1:],np.array([finger2Val]))
		self.finger1FilteredData = scipy.signal.lfilter(self.b, self.a, self.finger1RawData)
		self.finger2FilteredData = scipy.signal.lfilter(self.b, self.a, self.finger2RawData)
		self.fingerMax = self.finger1FilteredData.max() + self.finger2FilteredData.max()
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

