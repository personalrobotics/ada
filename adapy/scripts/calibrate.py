#!/usr/bin/env python
"""
Provides a simple tool for finding the transform on the JACO between the hand
(link 6) and the IR camera.
"""

import IPython
import adapy
import argparse
import logging
import numpy as np
import openravepy
import rospy
import kinbody_detector.apriltag_kinbody as ak
from tf.transformations import *
import tf
from visualization_msgs.msg import MarkerArray

class TransformEstimator:
    def __init__(self,topic,env,robot):
        self.env = env
        self.robot = robot
        self.br = tf.TransformBroadcaster()
        self.updating = False
        rospy.Subscriber(topic, MarkerArray, self.marker_callback)
        self.marker_pub = rospy.Publisher('calibration/marker_array', MarkerArray, queue_size=10)
        self.displaying = False
        self.possible_transforms = []

    def start(self):
        self.possible_transforms=[]
        self.updating = True

    def stop(self):
        self.updating = False

    def marker_callback(self,msg):
        self.last_msg = msg
        kinbody_names = [x.GetName() for x in self.env.GetBodies()]
        previously_detected_tags = [x for x in kinbody_names if 'detected' in x]

        tags = msg.markers
        hand = robot.GetLink('j2n6a300_link_6')
        hand_transform = hand.GetTransform()
        detected_tags = []
        for tag in tags:
            tag_id = tag.id
            kinbody_name = 'tag' + str(tag_id)
            kinbody = env.GetKinBody(kinbody_name)
            if kinbody:
                hand_in_world = hand_transform
                tag_in_world = kinbody.GetTransform()
                # convert from pose
                #tag_in_camera = rotation_matrix(angle=tag.pose.orientation.w,
                #                                direction=[tag.pose.orientation.x, 
                #                                           tag.pose.orientation.y,
                #                                           tag.pose.orientation.z])
                trans = translation_matrix(np.array([tag.pose.position.x,tag.pose.position.y,tag.pose.position.z]))
                rot = quaternion_matrix(np.array([tag.pose.orientation.x,tag.pose.orientation.y,tag.pose.orientation.z,tag.pose.orientation.w]))
                tag_in_camera = np.dot(trans, rot)
                
                #tag_in_camera[0,3] = tag.pose.position.x
                #tag_in_camera[1,3] = tag.pose.position.y
                #tag_in_camera[2,3] = tag.pose.position.z

                # Now to the approximation.
                camera_in_world = np.dot(tag_in_world, inverse_matrix(tag_in_camera))
                camera_in_hand = np.dot(inverse_matrix(hand_in_world), camera_in_world)
                if self.updating:
                    self.possible_transforms.append(camera_in_hand)
                
                # Create a "detected tag" kinbody in openrave if does not exist
                detected_tag_name = 'detected_tag' + str(tag_id)
                detected_tags.append(detected_tag_name)

                if not detected_tag_name in kinbody_names:
                    detected_tag_kb_path = ak.BuildApriltagKinbody(family=(36,11),
                                      index=tag_id,
                                      width=0.06,
                                      thickness=0.001,
                                      name=detected_tag_name,
                                      path='/tmp')
                    detected_tag_kinbody = env.ReadKinBodyXMLFile(tag_kb_path)
                    # Set pose based on message here
                    detected_tag_kinbody.SetName(detected_tag_name)
                    
                    detected_tag_kinbody.SetTransform(np.dot(tag_in_camera, self.robot.GetLink('Camera_Depth_Frame').GetTransform()))
                    self.env.Add(detected_tag_kinbody)
                else:
                    detected_tag_kinbody = self.env.GetKinBody(detected_tag_name)
                    detected_tag_kinbody.SetTransform(np.dot(tag_in_camera, self.robot.GetLink('Camera_Depth_Frame').GetTransform()))
        # Delete tags no longer detected
        for old in previously_detected_tags:
            if not old in detected_tags:
                env.Remove(env.GetKinBody(old))
                    
        if self.displaying:
            q = quaternion_from_matrix(self.candidate_transform)
            trans = translation_from_matrix(self.candidate_transform)
            self.br.sendTransform(trans, q, rospy.Time.now(),'candidate_camera_frame','j2n6a300_link_6')
            new_marker_msg = msg
            for marker in new_marker_msg.markers:
                marker.header.frame_id = 'candidate_camera_frame'
                marker.color.r = 0
                marker.color.b = 1
                marker.color.g = 0
            self.marker_pub.publish(new_marker_msg)


    def display(self):
        self.displaying=True
        for possible_transform in self.possible_transforms:
            self.candidate_transform = possible_transform
            command = raw_input('Press "n" for next transform\nPress "p" to print\nPress "q" to quit\n')
            if command == 'q':
                self.displaying=False
                return
            if command == 'p':
                pose = translation_from_matrix(self.candidate_transform)
                q = quaternion_from_matrix(self.candidate_transform)
                euler = euler_from_matrix(self.candidate_transform)
                print 'Pose: %f %f %f' % (pose[0], pose[1], pose[2])
                print 'Orientation (x,y,z,w): %f %f %f %f' % (q[0],q[1],q[2],q[3])
                print 'Orientation (rpy): %f %f %f' % (euler[0], euler[1], euler[2])
                continue
            if command == 'n':
                continue
        print 'Went through all transforms!'
        self.displaying = False



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='utility script for loading AdaPy')
    parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
    parser.add_argument('-v', '--viewer', nargs='?', const=True,
                        help='attach a viewer of the specified type')
    parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
    parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
    args = parser.parse_args()

    openravepy.RaveInitialize(True)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    if not args.sim:
        rospy.init_node('adapy', anonymous=True)

    env, robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )

    thickness = 0.001
    family='36h11'
    width=0.06
    indices = range(100,140)
    spacing = 0.18

    for index in indices:
        # Generate and place the tags where they should be:
        tag_kb_path = ak.BuildApriltagKinbody(family=(36,11),
                                          index=index,
                                          width=width,
                                          thickness=thickness,
                                          name='tag' + str(index),
                                          path='/tmp')
        tag_kinbody = env.ReadKinBodyXMLFile(tag_kb_path)
        
        # flip tag so the correct side is facing up
        t = rotation_matrix(angle=math.pi, direction=np.array([0,0,1]))
        # then translate based on tag id
        if index<120:
            row = np.floor((index-100)/4)
            col = (index-100)%4
            t[0,3] = spacing*(4-col)-spacing/2
            t[1,3] = -spacing*row
        else:
            row = np.floor((index-120)/4)
            col = (index-120)%4
            t[0,3] = -(spacing*(col)+spacing/2)
            t[1,3] = -spacing*row
            print 'Tag %i is in row %i and col %i' % (index, row, col)
        # table offset
        t[2,3] = -0.02
        tag_kinbody.SetTransform(t)
        env.Add(tag_kinbody)


    transform_estimator = TransformEstimator('/apriltags/marker_array', env, robot)

    IPython.embed()
