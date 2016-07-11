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

        self.rawData = np.array([]) 
        self.filteredData = np.array([]) 
        
        # Butterworth digital and analog filter design.
        self.b, self.a = scipy.signal.butter(4, 0.5, 'low', analog=False)
        # self.objPub = rospy.Publisher('object_grasp_flag',Bool, queue_size = 1)
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.fingerEffortCallback)
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
        if len(self.rawData) == 0:
            self.rawData = np.array([data.effort])
        else:
            self.rawData = np.vstack([self.rawData, np.array([data.effort])])

        print len(self.rawData)
        # import IPython;IPython.embed()
        if len(self.rawData) >= 3500:
            import csv
            with open('./data_no_filtered.csv', 'w') as outcsv:   
                writer = csv.writer(outcsv, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL, lineterminator='\n')
                # writer.writerow(['number', 'text', 'number'])
                for row in self.rawData:
                    writer.writerow(row)

            self.filteredData = scipy.signal.lfilter(self.b, self.a, self.rawData)
            with open('./data_filtered.csv', 'w') as outcsv:   
                writer = csv.writer(outcsv, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL, lineterminator='\n')
                # writer.writerow(['number', 'text', 'number'])
                for row in self.filteredData:
                    writer.writerow(row)


            import IPython;IPython.embed()

            # self.subscriber.unregister()


def main():
    print "Beginning node"
    rospy.init_node('GraspVerify')
    DetectObjectGrasp()
    print 'stop2'
    sys.exit(0)

if __name__ == "__main__":
    main()

