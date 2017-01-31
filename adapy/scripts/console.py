#!/usr/bin/env python
"""
Provides a simple console that sets up basic functionality for 
using AdaPy and openravepy.
"""

import IPython
import adapy
import argparse
import logging
import numpy
import openravepy
import rospy

import threading
import tf

def publish_tf(robot, br):
    
    # Make world and map the same (for rendering purposes)
    br.sendTransform((0, 0, 0),
                     (0, 0, 0, 1),
                     rospy.Time.now(),
                     'world',
                     'map')
    
    # Loop over all links and publish transforms
    for link in robot.GetLinks():
        parents = link.GetParentLinks()
        for p in parents:
            l_T = link.GetTransform()
            p_T = p.GetTransform()
            T = numpy.dot(numpy.linalg.inv(p_T), l_T)
            l_frame = link.GetName()
            p_frame = p.GetName()
            br.sendTransform(tf.transformations.translation_from_matrix(T),
                             tf.transformations.quaternion_from_matrix(T),
                             rospy.Time.now(),
                             l_frame,
                             p_frame)
    if not rospy.is_shutdown():   
        new_tf_thread = threading.Timer(1, publish_tf, (robot, br))
        new_tf_thread.start()

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

    #if not args.sim:
    rospy.init_node('adapy', anonymous=True)

    env, robot = adapy.initialize(
        sim=args.sim,
        attach_viewer=args.viewer,
        env_path=args.env_xml
    )
    
    if args.sim:
        # Create a tf broadcaster
        br = tf.TransformBroadcaster()
        
        # Start a thread to publish the tf values
        tf_thread = threading.Thread(target=publish_tf, args=(robot,br))
        tf_thread.start()
        

    IPython.embed()
    rospy.signal_shutdown('Finished running console.py')
