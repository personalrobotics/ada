PACKAGE = 'adapy'
import logging
import prpy
from catkin.find_in_workspaces import find_in_workspaces
from prpy.base.mico import Mico
from prpy.base.micohand import MicoHand
from prpy.base.micorobot import MicoRobot

logger = logging.getLogger(PACKAGE)

class ADARobot(MicoRobot):
    def __init__(self, sim):
        import os.path
        from rospkg import RosPack

        # We need to hard-code the name. Otherwise, it defaults to
        # "mico-modified".
        MicoRobot.__init__(self, robot_name='ada')

        # Absolute path to this package.
        ros_pack = RosPack()
        package_path = ros_pack.get_path(PACKAGE)

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('Mico')
        self.arm.hand = self.arm.GetEndEffector()
        self.manipulators = [ self.arm ]

        # Bind robot-specific subclasses.
        prpy.bind_subclass(self.arm, Mico, sim=sim,
                           controller_namespace='/mico_controller')
        prpy.bind_subclass(self.arm.hand, MicoHand, sim=sim,
                           manipulator=self.arm)

        # Support for named configurations.
        self.configurations.add_group('arm', self.arm.GetIndices())

        try:
            configurations_path = os.path.join(
                    package_path, 'config/configurations.yaml')
            self.configurations.load_yaml(configurations_path)
        except IOError as e:
            logger.warning('Failed loading named configurations from "%s": %s',
                           configurations_path, e.message)


        if self.tsrlibrary is not None:
            tsr_paths_relative = [
                'config/glass_grasp_tsr.yaml',
                'config/glass_move_tsr.yaml'
            ]

            for tsr_path_relative in tsr_paths_relative:
                tsr_paths = find_in_workspaces(
                    search_dirs=['share'], project='adapy',
                    path=tsr_path_relative, first_match_only=True
                )

                if not tsr_paths:
                    raise ValueError(
                        'Unable to load named tsrs from path "%s".'.format(
                            tsr_path_relative))

                tsr_path = tsr[0]

                try:
                    self.tsrlibrary.load_yaml(tsr_path)
                except IOError as e:
                    raise ValueError(
                        'Failed loading TSRs from "{:s}": {:s}'.format(
                            tsr_path, e.message))


        # Initialize a default planning pipeline.
        from prpy.planning import Sequence, Ranked
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


        self.snap_planner = SnapPlanner()
        self.cbirrt_planner = CBiRRTPlanner()
        self.vectorfield_planner = VectorFieldPlanner()
        self.greedyik_planner = GreedyIKPlanner()
        self.planner = Sequence(self.cbirrt_planner)

    def CloneBindings(self, parent):
        from prpy import Cloned

        MicoRobot.CloneBindings(self, parent)

        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
