PACKAGE = 'adapy'
import logging, numpy, openravepy, rospy, time
import prpy

logger = logging.getLogger('adapy')

# Absolute path to this package.
from rospkg import RosPack
ros_pack = RosPack()
PACKAGE_PATH = ros_pack.get_path(PACKAGE)

class ADARobot(prpy.base.MicoRobot):
    def __init__(self, arm_sim, perception_sim):
        prpy.base.MicoRobot.__init__(self)

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('Mico')
        self.arm.hand = self.arm.GetEndEffector()
        self.manipulators = [ self.arm ]
        
        # Dynamically switch to self-specific subclasses.
        from prpy.base import MicoHand, Mico
        prpy.bind_subclass(self.arm, Mico, sim=arm_sim, controller_namespace='/mico_controller')
        prpy.bind_subclass(self.arm.hand, MicoHand, sim=arm_sim, manipulator=self.arm,
                           controller_namespace='/mico_hand_controller', hand_namespace='/mico_hand')
        
        # Support for named configurations.
        import os.path
        self.configurations.add_group('arm', self.arm.GetArmIndices())
        
        try:
            configurations_path = os.path.join(PACKAGE_PATH, 'config/configurations.yaml')
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            logger.warning('Failed loading named configurations from %s.', configurations_path)

        # Initialize a default planning pipeline.
        from prpy.planning import Planner, Sequence, Ranked #, Fastest
        from prpy.planning import CBiRRTPlanner, CHOMPPlanner, MKPlanner, SnapPlanner
        self.cbirrt_planner = CBiRRTPlanner()
        self.chomp_planner = CHOMPPlanner()
        self.mk_planner = MKPlanner()
        self.snap_planner = SnapPlanner()
        #self.planner = Sequence(self.snap_planner, Ranked(self.chomp_planner, Sequence(self.mk_planner, self.cbirrt_planner)))
        #self.planner = Sequence(self.snap_planner, Ranked(self.cbirrt_planner))
        self.planner = Sequence(self.cbirrt_planner)

        # Setting necessary sim flags
        # TODO: define this part properly
        '''
        self.perception_sim = perception_sim
        if not self.perception_sim:
          args = 'ADASensorSystem adapy /ada_perception/ map'
          self.ada_sensorsystem = openravepy.RaveCreateSensorSystem(self.GetEnv(), args)
          if self.ada_sensorsystem is None:
            raise Exception("creating the ADAPerception sensorsystem failed")
        '''
	
    def CloneBindings(self, parent):
        from prpy import Cloned
        prpy.base.MicoRobot.CloneBindings(self, parent)
        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
