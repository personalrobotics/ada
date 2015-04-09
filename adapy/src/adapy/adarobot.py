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

class ADARobot(Robot):
    def __init__(self, sim):
        from prpy.base.mico import Mico
        from prpy.base.micohand import MicoHand
        from util import AdaPyException, find_adapy_resource

        # We need to hard-code the name. Otherwise, it defaults to
        # "mico-modified".
        Robot.__init__(self, robot_name='ada')

        # Convenience attributes for accessing self components.
        self.arm = self.GetManipulator('Mico')
        self.arm.hand = self.arm.GetEndEffector()
        self.manipulators = [ self.arm ]

        # Bind robot-specific subclasses.
        prpy.bind_subclass(self.arm, Mico, sim=sim)
        prpy.bind_subclass(self.arm.hand, MicoHand, sim=sim,
                           manipulator=self.arm)

        # TODO: Load an IdealController controller in simulation.

        # Load default named configurations from YAML.
        self.configurations.add_group('arm', self.arm.GetIndices())
        try:
            self.configurations.load_yaml(
                find_adapy_resource(CONFIGURATIONS_PATH))
        except IOError as e:
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

        actual_planner = Sequence(
            self.snap_planner,
            self.cbirrt_planner
        )
        self.planner = FirstSupported(
            actual_planner,
            NamedPlanner(delegate_planner=actual_planner)
        )

    def CloneBindings(self, parent):
        super(ADARobot, self).CloneBindings(parent)

        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner
