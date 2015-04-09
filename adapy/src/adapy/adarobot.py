PACKAGE = 'adapy'
import logging
import prpy
from prpy import Cloned
from prpy.base.robot import Robot

logger = logging.getLogger(PACKAGE)

CONFIGURATIONS_PATH = 'config/configurations.yaml'
TSR_PATHS = [
    'config/glass_grasp_tsr.yaml',
    'config/glass_move_tsr.yaml',
]


def find_adapy_resource(relative_path):
    from catkin.find_in_workspaces import find_in_workspaces

    paths = find_in_workspaces(project='adapy', search_dirs=['share'],
                               path=relative_path, first_match_only=True)

    if paths and len(paths) == 1:
        return paths[0]
    else:
        raise IOError('Loading AdaPy resource "{:s}" failed.'.format(
                      relative_path))


class ADARobot(Robot):
    def __init__(self, sim):
<<<<<<< HEAD
        from prpy.base.mico import Mico
        from prpy.base.micohand import MicoHand
=======
        MicoRobot.__init__(self, robot_name = 'ADA')
>>>>>>> master

        # We need to hard-code the name. Otherwise, it defaults to
        # "mico-modified".
        Robot.__init__(self, robot_name='ada')

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('Mico')
        self.arm.hand = self.arm.GetEndEffector()
        self.manipulators = [ self.arm ]

<<<<<<< HEAD
        # Bind robot-specific subclasses.
        prpy.bind_subclass(self.arm, Mico, sim=sim)
        prpy.bind_subclass(self.arm.hand, MicoHand, sim=sim,
                           manipulator=self.arm)

        # TODO: Load an IdealController controller in simulation.
=======
>>>>>>> master

        # Load default named configurations from YAML.
        self.configurations.add_group('arm', self.arm.GetIndices())
        try:
            self.configurations.load_yaml(
                find_adapy_resource(CONFIGURATIONS_PATH))
        except IOError as e:
<<<<<<< HEAD
            logger.warning('Failed loading named configurations from "%s": %s',
                           configurations_path, e.message)

        # Load default TSRs from YAML.
        for tsr_path_relative in TSR_PATHS:
            try:
                tsr_path = find_adapy_resource(tsr_path_relative)
                self.tsrlibrary.load_yaml(tsr_path)
            except IOError as e:
                raise ValueError(
                    'Failed loading TSRs from "{:s}": {:s}'.format(
                        tsr_path, e.message))

        # Initialize the default planning pipeline.
        from prpy.planning import Sequence, Ranked, FirstSupported
        from prpy.planning import (
=======
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
>>>>>>> master
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

<<<<<<< HEAD
        self.snap_planner = SnapPlanner()
        self.cbirrt_planner = CBiRRTPlanner()
        self.vectorfield_planner = VectorFieldPlanner()
        self.greedyik_planner = GreedyIKPlanner()

        actual_planner = Sequence(
            self.snap_planner,
            self.cbirrt_planner
        )
        self.planner = FirstSupported(
            actual_planner,
            NamedPlanner(delegate_planner=actual_planner)
        )
=======
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
  
>>>>>>> master

    def CloneBindings(self, parent):
        super(ADARobot, self).CloneBindings(parent)

        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
