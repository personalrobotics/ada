PACKAGE = 'adapy'
import logging, prpy
import openravepy
logger = logging.getLogger(PACKAGE)

class ADARobot(prpy.base.MicoRobot):
    def __init__(self, sim):
        prpy.base.MicoRobot.__init__(self)

        # Absolute path to this package.
        from rospkg import RosPack
        ros_pack = RosPack()
        package_path = ros_pack.get_path(PACKAGE)

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('Mico')
        self.arm.hand = self.arm.GetEndEffector()
        self.manipulators = [ self.arm ] #, self.head ]

        # TODO: Add the head.
        # TODO: Add the hands.
        # Dynamically switch to self-specific subclasses.
        from prpy.base import MicoHand, Mico
        prpy.bind_subclass(self.arm, Mico, sim=sim, controller_namespace='/mico_controller')
        #prpy.bind_subclass(self.arm.hand, MicoHand, sim=sim, manipulator = self.arm,  controller_namespace='/mico_hand_controller', hand_namespace='/mico_hand')
        #prpy.bind_subclass(self.left_hand, R2Hand, manipulator=self.left_arm, sim=sim)
        #prpy.bind_subclass(self.right_hand, R2Hand, manipulator=self.right_arm, sim=sim)
        #prpy.bind_subclass(self.head, R2Head, sim=sim)
        #print "*******************hello!*********************"

        # Create a controller and (optionally) connect it to the real hardware.
        #if sim:
        #    controller_args = 'IdealController'
        #else:
        #    controller_args = ("roscontroller openrave {0} 1".format(self.sup.ns))

        #if self.controller is None:
        #    raise openravepy.openrave_exception(
        #        'Creating controller "{:s}" failed.'.format(controller_args))

        #self.controller = openravepy.RaveCreateController(self.GetEnv(), controller_args)
        #controlled_dofs = range(self.GetDOF())
        #self.SetController(self.controller, controlled_dofs, 0)

        # Support for named configurations.
        import os.path
        self.configurations.add_group('arm', self.arm.GetIndices())

        try:
            import os.path
            configurations_path = os.path.join(package_path, 'config/configurations.yaml')
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            logger.warning('Failed loading named configurations from "%s".', configurations_path)

        # Initialize a default planning pipeline.
        from prpy.planning import Planner, Sequence, Ranked
        from prpy.planning import (CBiRRTPlanner, CHOMPPlanner, IKPlanner, OMPLPlanner,
                                   MKPlanner, NamedPlanner, SnapPlanner)
        self.cbirrt_planner = CBiRRTPlanner()
        self.chomp_planner = CHOMPPlanner()
        self.mk_planner = MKPlanner()
        self.snap_planner = SnapPlanner()
        self.named_planner = NamedPlanner()
        self.ompl_planner = OMPLPlanner('RRTConnect')
        self.ik_planner = IKPlanner()
        self.planner = Sequence(self.cbirrt_planner)
        #self.ik_planner,
                                #self.named_planner
                                #self.snap_planner)
                                #self.mk_planner)
                                #self.ompl_planner)
                                #self.cbirrt_planner)

    """
    def ExecuteTrajectory(self, traj, retime=True, **kw_args):
        if retime:
            # Smooth the trajectory using quadratic interpolation.
            result = openravepy.planningutils.SmoothTrajectory(traj, 0.95, 0.95, '', '')
            if result != openravepy.PlannerStatus.HasSolution:
                raise ValueError('Retiming trajectory failed.')

            import prpy.rave
            num_fixed = prpy.rave.fix_trajectory(traj)
            if num_fixed > 0:
                logger.warning('Removed %d invalid waypoints from trajectory.', num_fixed)

        self.GetController().SetPath(traj)
        return traj
    """
  

    def CloneBindings(self, parent):
        from prpy import Cloned
        prpy.base.MicoRobot.CloneBindings(self, parent)
        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
