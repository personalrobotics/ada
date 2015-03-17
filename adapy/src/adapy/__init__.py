#!/usr/bin/env python
PACKAGE = 'adapy'
import logging, prpy, openravepy, adapy

logger = logging.getLogger('adapy')

# Add dependencies to our OpenRAVE plugin and data search paths.


def initialize(robot_xml=None, env_path=None, attach_viewer=False, **kw_args):
    prpy.logger.initialize_logging()

    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
      if not env.Load(env_path):
        raise Exception('Unable to load environment frompath %s' % env_path)

  
    if robot_xml is None:
       import os, rospkg
       rospack = rospkg.RosPack()
       base_path = rospack.get_path('ada_description')

       robot_xml = os.path.join(base_path, 'ordata', 'robots', 'mico-modified.robot.xml')

    robot = env.ReadRobotXMLFile(robot_xml)
    env.Add(robot)


    

    # Default arguments.
    #keys = [ 'sim']
    #for key in keys:
     #   if key not in kw_args:
     #      kw_args[key] = sim

    from adarobot import ADARobot
    prpy.bind_subclass(robot, ADARobot, **kw_args)

#     from r2robot import R2Robot
#     prpy.bind_subclass(robot, R2Robot, **kw_args)

#     # Start by attempting to load or_rviz.
    if attach_viewer == True:
        attach_viewer = 'rviz'
        env.SetViewer(attach_viewer)

#         # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
           logger.warning('Loading rviz failed. Falling back on qt_coin.')
           attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            raise Exception('Failed creating viewer of type "{0:s}".'.format(attach_viewer))
     
#     # Remove the ROS logging handler again. It might have been added when we loaded or_rviz.
    prpy.logger.remove_ros_logger()
    #print "hello!!!!!************************"

    return env, robot

