PACKAGE = 'adapy'
#import roslib; roslib.load_manifest(PACKAGE)
#import openrave_exports; openrave_exports.export(PACKAGE)
import prpy, openravepy, logging
import math

# Add dependencies to our OpenRAVE plugin and data search paths.
import prpy.dependency_manager
#prpy.dependency_manager.export(PACKAGE)


logger = logging.getLogger('adapy')

def initialize(robot_xml=None, env_path=None, attach_viewer=False, sim=True, **kw_args):
    # Create the environment.
    env = openravepy.Environment()
    if env_path is not None:
        if not env.Load(env_path):
            raise Exception('Unable to load environment frompath %s' % env_path)

    if robot_xml is None:
        import os, rospkg
        rospack = rospkg.RosPack()
        base_path = rospack.get_path('ada_description')
        #robot_xml = os.path.join(base_path, 'ordata', 'robots', 'mico_zero_pose.robot.xml')
        robot_xml = os.path.join(base_path, 'ordata', 'robots', 'mico-modified.robot.xml')

    robot = env.ReadRobotXMLFile(robot_xml)
    env.Add(robot)
    
    #FIXME: we won't need this once the URDF is fixed
    # NO LONGER NEEDED -- offset moved to hardware driver (for now)
    # at some point, we can change the zero pose (in both the model and the driver)
    #with env:
    #  robot.SetDOFValues([-math.pi,math.pi/2,-math.pi/2,math.pi,math.pi,0],[0,1,2,3,4,5],openravepy.KinBody.CheckLimitsAction.Nothing)
    #  robot.SetZeroConfiguration()
    
    prpy.logger.initialize_logging()

    # Default arguments.
    keys = [ 'arm_sim', 'perception_sim' ]
    for key in keys:
        if key not in kw_args:
            kw_args[key] = sim

    
    from adarobot import ADARobot
    prpy.bind_subclass(robot, ADARobot, **kw_args)

    if attach_viewer == True:
        # Start by attempting to load or_rviz.
        attach_viewer = 'or_rviz'
        env.SetViewer(attach_viewer)

        # Fall back on qtcoin if loading or_rviz failed
        if env.GetViewer() is None:
            logger.warning('Loading or_rviz failed. Falling back on qt_coin.')
            attach_viewer = 'qtcoin'

    if attach_viewer and env.GetViewer() is None:
        env.SetViewer(attach_viewer)
        if env.GetViewer() is None:
            raise Exception('Failed creating viewer of type "{0:s}".'.format(attach_viewer))
        

    return env, robot
