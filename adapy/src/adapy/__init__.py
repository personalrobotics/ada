#!/usr/bin/env python
PACKAGE = 'adapy'
import logging, prpy, openravepy, adapy

logger = logging.getLogger('adapy')

# Add dependencies to our OpenRAVE plugin and data search paths.


def initialize(robot_xml=None, env_path=None, attach_viewer=False, load_xml = True, **kw_args):
    prpy.logger.initialize_logging()

    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
      if not env.Load(env_path):
        raise Exception('Unable to load environment frompath %s' % env_path)

    if load_xml is True:
      if robot_xml is None:
        import os, rospkg
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('ada_description')

        robot_xml = os.path.join(base_path, 'ordata', 'robots', 'mico-modified.robot.xml')

      robot = env.ReadRobotXMLFile(robot_xml)
      env.Add(robot)
    else:
      if prpy.dependency_manager.is_catkin():
            # Find the HERB URDF and SRDF files.
          from catkin.find_in_workspaces import find_in_workspaces
          share_directories = find_in_workspaces(search_dirs=['share'],
                                                   project='ada_description')
          if not share_directories:
              logger.error('Unable to find the Ada model. Do you have the'
                             ' package ada installed?')
              raise ValueError('Unable to find Ada model.')


          import os.path, rospkg
          found_models = False
          rospack = rospkg.RosPack()
          base_path = rospack.get_path('ada_description')

          for share_directory in share_directories:

              urdf_path = os.path.join(base_path, 'robots', 'mico-modified-updated.urdf')
              srdf_path = os.path.join(base_path, 'robots', 'mico-modified.srdf')

              if os.path.exists(urdf_path) and os.path.exists(srdf_path):
                  found_models = True
                  break
            #from IPython import embed
            #embed()
          if not found_models:
              logger.error('Missing URDF file for ADA.'
                           ' Is the ada_description package properly installed?')
              raise ValueError('Unable to find ADA URDF file.')

            # Load the URDF file into OpenRAVE.

          urdf_module = openravepy.RaveCreateModule(env, 'urdf')
          if urdf_module is None:
              logger.error('Unable to load or_urdf module. Do you have or_urdf'
                             ' built and installed in one of your Catkin workspaces?')
              raise ValueError('Unable to load or_urdf plugin.')


          args = 'Load {:s} {:s}'.format(urdf_path, srdf_path)
            # ada_name = urdf_module.SendCommand(args)
            # if ada_name is None:
            #     raise ValueError('Failed loading ADA model using or_urdf.')
            #rom IPython import embed
            #embed()
          name = urdf_module.SendCommand(args)
          robot = env.GetRobot(name)
          if robot is None:
              raise ValueError('Unable to find robot with name "{:s}".'.format(
                                 ada_name))   
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

