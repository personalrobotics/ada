PACKAGE = 'adapy'
import logging, prpy
import openravepy
from prpy.base.mico import Mico
from prpy.base.micohand import MicoHand
from prpy.base.micorobot import MicoRobot

logger = logging.getLogger(PACKAGE)

class ADARobot(MicoRobot):
    def __init__(self, sim):
        MicoRobot.__init__(self)

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
        prpy.bind_subclass(self.arm, Mico, sim=sim, controller_namespace='/mico_controller')
        #if(sim == True): #for now this works only in simulation
        #   prpy.bind_subclass(self.arm.hand, MicoHand, sim=sim, manipulator = self.arm,  controller_namespace='/mico_controller', hand_namespace='/mico_hand')
        prpy.bind_subclass(self.arm.hand, MicoHand, sim=sim, manipulator = self.arm)

        #prpy.bind_subclass(self.left_hand, R2Hand, manipulator=self.left_arm, sim=sim)


        # Support for named configurations.
        import os.path
        self.configurations.add_group('arm', self.arm.GetIndices())

        try:
            import os.path
            configurations_path = os.path.join(package_path, 'config/configurations.yaml')
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            logger.warning('Failed loading named configurations from "%s".', configurations_path)

                # Support for loading tsrs from yaml
        if prpy.dependency_manager.is_catkin():
            from catkin.find_in_workspaces import find_in_workspaces
            tsrs_paths = find_in_workspaces(search_dirs=['share'], project='adapy',
                             path='config/tsrs.yaml', first_match_only=True)
            if not tsrs_paths:
                raise ValueError('Unable to load named tsrs from "config/tsrs.yaml".')

            tsrs_path = tsrs_paths[0]
        else:
            tsrs_path = os.path.join(package_path, 'config/tsrs.yaml')

        try:
            self.tsrlibrary.load_yaml(tsrs_path)
        except IOError as e:
            raise ValueError('Failed loading named tsrs from "{:s}".'.format(
                tsrs_path))


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
        self.planner = Sequence(
        #self.cbirrt_planner,
        #self.ik_planner,
                                #self.named_planner
                                #self.snap_planner)
                                #self.mk_planner)
                                #self.ompl_planner)
                                self.cbirrt_planner)

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
        MicoRobot.CloneBindings(self, parent)
        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
