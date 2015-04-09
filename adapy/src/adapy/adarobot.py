PACKAGE = 'adapy'
import logging, prpy
import openravepy
from prpy.base.mico import Mico
from prpy.base.micohand import MicoHand
from prpy.base.micorobot import MicoRobot

logger = logging.getLogger(PACKAGE)

class ADARobot(MicoRobot):
    def __init__(self, sim):
        MicoRobot.__init__(self, robot_name = 'ADA')

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


        # Support for named configurations.
        import os.path
        self.configurations.add_group('arm', self.arm.GetIndices())

        try:
            import os.path
            configurations_path = os.path.join(package_path, 'config/configurations.yaml')
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            logger.warning('Failed loading named configurations from "%s".', configurations_path)


        if self.tsrlibrary is not None:
            tsrPaths = ['config/glass_grasp_tsr.yaml', 'config/glass_move_tsr.yaml']
            for tsrPath in tsrPaths:
              if prpy.dependency_manager.is_catkin():
                  from catkin.find_in_workspaces import find_in_workspaces
                  tsrs_paths = find_in_workspaces(search_dirs=['share'], project='adapy',
                                 path=tsrPath, first_match_only=True)
                  if not tsrs_paths:
                    raise ValueError('Unable to load named tsrs from path "%s".'. tsrPath)

                  tsrs_path = tsrs_paths[0]
              else:
                  tsrs_path = os.path.join(package_path, tsrPath)

              try:
                self.tsrlibrary.robot_name = 'ADA' #need to hardcode this, otherwise the name becomes mico-modified
                self.tsrlibrary.load_yaml(tsrs_path)
                #self.tsrlibrary.load_yaml(tsrs_path)
              except IOError as e:
                  raise ValueError('Failed loading named tsrs from "{:s}".'.format(
                    tsrs_path))



        # Initialize a default planning pipeline.
        from prpy.planning import (
            FirstSupported,
            MethodMask,
            Ranked,
            Sequence,
        )
        from prpy.planning import (
            BiRRTPlanner,
            CBiRRTPlanner,
            CHOMPPlanner,
            GreedyIKPlanner,
            IKPlanner,
            NamedPlanner,
            SBPLPlanner,
            SnapPlanner,
            TSRPlanner,
            VectorFieldPlanner
        )

        # TODO: These should be meta-planners.
        self.named_planner = NamedPlanner()
        self.ik_planner = IKPlanner()

        # Special-purpose planners.
        self.snap_planner = SnapPlanner()
        self.vectorfield_planner = VectorFieldPlanner()
        self.greedyik_planner = GreedyIKPlanner()

        # General-purpose planners.
        self.birrt_planner = BiRRTPlanner()
        self.cbirrt_planner = CBiRRTPlanner()

        actual_planner = Sequence(
        #     # First, try the straight-line trajectory.
              self.snap_planner,
        #     # Then, try a few simple (and fast!) heuristics.
              self.vectorfield_planner,
              self.greedyik_planner,
        #     # Next, try a trajectory optimizer.
        #     #self.trajopt_planner or self.chomp_planner,
        #     # If all else fails, call an RRT.
             self.birrt_planner,
             MethodMask(
                 FirstSupported(
        #             # Try sampling the TSR and planning with BiRRT. This only
        #             # works for PlanToIK and PlanToTSR with strictly goal TSRs.
                     TSRPlanner(delegate_planner=self.birrt_planner),
        #             # Fall back on CBiRRT, which also handles start and
        #             # constraint TSRs.
                     self.cbirrt_planner,
                 ),
                 methods=['PlanToIK', 'PlanToTSR', 'PlanToEndEffectorPose', 'PlanToEndEffectorOffset']
             )
         )
        self.planner = FirstSupported(
            actual_planner,
             # Special purpose meta-planner.
             NamedPlanner(delegate_planner=actual_planner),
        )
       # self.planner = self.cbirrt_planner
        #from herbpy.action import *
        from adapy.tsr import * 
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