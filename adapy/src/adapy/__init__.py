#!/usr/bin/env python
import logging, prpy, openravepy, adapy

URDF_PATH = 'robots/mico-modified-updated.urdf'
URDF_PACKAGE = 'ada_description'
SRDF_PATH = ''
SRDF_PACKAGE = 'ada_description'

def initialize(env_path=None, attach_viewer=False, **kw_args):
    from adarobot import ADARobot
    from util import find_adapy_resource

    prpy.logger.initialize_logging()

    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
        if not env.Load(env_path):
            raise IOError(
                'Unable to load environment "{:s}".'.format(env_path))

    # Load ADA from URDF.
    urdf_path = find_adapy_resource(URDF_PATH, package=URDF_PACKAGE)
    if urdf_path is None:
        raise AdaPyException('Failed loading URDF "{:s}". Do you have the'
                             ' package {:s} installed?'.format(
                                URDF_PATH, URDF_PACKAGE))

    srdf_path = ''
    if srdf_path is None:
        raise AdaPyException('Failed loading SRDF "{:s}". Do you have the'
                             ' package {:s} installed?'.format(
                                SRDF_PATH, SRDF_PACKAGE))

    # Use or_urdf to load ADA from URDF and SRDF.
    with env:
        or_urdf = openravepy.RaveCreateModule(env, 'urdf')
        ada_name = or_urdf.SendCommand(
            'load {:s} {:s}'.format(urdf_path, srdf_path))

    robot = env.GetRobot(ada_name)
    if robot is None:
        raise AdaPyException('Failed loading ADA with or_urdf.')

    # Bind AdaPy-specific functionality on the robot.
    prpy.bind_subclass(robot, ADARobot, **kw_args)

    # Start by attempting to load or_rviz.
    if attach_viewer == True:
        attach_viewer = 'rviz'
        env.SetViewer(attach_viewer)

        # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
           logger.warning('Loading rviz failed. Falling back on qt_coin.')
           attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            raise AdaPyException(
                'Failed creating viewer of type "{:s}".'.format(attach_viewer))
     
    # Remove the ROS logging handler again. It might have been added when we
    # loaded or_rviz.
    prpy.logger.remove_ros_logger()

    return env, robot

