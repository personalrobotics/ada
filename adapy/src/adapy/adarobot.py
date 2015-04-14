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

        self.simulated = sim

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

        # Create ros_control clients for execution on real hardware.
        if not sim:
            from .controller_client import (ControllerManagerClient,
                                            JointStateClient)
            from .trajectory_client import FollowJointTrajectoryClient

            self._trajectory_joint_names = [
                self.GetJointFromDOFIndex(i).GetName()
                for i in xrange(self.GetDOF())
            ]

            self._jointstate_client = JointStateClient(
                self, topic_name='/joint_states'
            )
            self._controller_client = ControllerManagerClient(
                ns='/controller_manager'
            )
            self._velocity_joint_mode_controller = self._controller_client.request(
                'velocity_joint_mode_controller'
            )
            self._velocity_joint_mode_controller.switch()

            self._trajectory_switcher = self._controller_client.request(
                'traj_controller'
            )
            self._trajectory_client = FollowJointTrajectoryClient(
                ns='/traj_controller/follow_joint_trajectory'
            )

        # Initialize the default planning pipeline.
        from prpy.planning import Sequence, Ranked, FirstSupported
        from prpy.planning import (
            BiRRTPlanner,
            CBiRRTPlanner,
            IKPlanner,
            GreedyIKPlanner,
            NamedPlanner,
            SBPLPlanner,
            SnapPlanner,
            TSRPlanner,
            VectorFieldPlanner
        )

        self.snap_planner = SnapPlanner()
        self.greedyik_planner = GreedyIKPlanner()
        self.cbirrt_planner = CBiRRTPlanner()
        self.vectorfield_planner = VectorFieldPlanner()

        actual_planner = Sequence(
            self.snap_planner,
            self.vectorfield_planner,
            self.greedyik_planner,
            self.cbirrt_planner
        )
        self.planner = FirstSupported(
            actual_planner,
            NamedPlanner(delegate_planner=actual_planner)
        )

        from prpy.planning.retimer import ParabolicSmoother
        self.simplifier = None
        self.smoother = ParabolicSmoother()

    def CloneBindings(self, parent):
        super(ADARobot, self).CloneBindings(parent)

        self.arm = Cloned(parent.arm)
        self.manipulators = [ self.arm ]
        self.planner = parent.planner

    def ExecuteTrajectory(self, traj, defer=False, timeout=None, switch=True,
                          unswitch=None, **kwargs):
        """ Executes a time trajectory on the robot.

        This function directly executes a timed OpenRAVE trajectory on the
        robot. If you have a geometric path, such as those returned by a
        geometric motion planner, you should first time the path using
        PostProcessPath. Alternatively, you could use the ExecutePath helper
        function to time and execute the path in one function call.

        If timeout = None (the default), this function does not return until
        execution has finished. Termination occurs if the trajectory is
        successfully executed or if a fault occurs (in this case, an exception
        will be raised). If timeout is a float (including timeout = 0), this
        function will return None once the timeout has ellapsed, even if the
        trajectory is still being executed.
        
        NOTE: We suggest that you either use timeout=None or defer=True. If
        trajectory execution times out, there is no way to tell whether
        execution was successful or not. Other values of timeout are only
        supported for legacy reasons.

        This function returns the trajectory that was actually executed on the
        robot, including controller error. If this is not available, the input
        trajectory will be returned instead.

        If switch = True, this function switches to the ros_control controllers
        necessary to execute traj. If unswitch = True, it also undoes this
        switch after the trajectory has finished executing. If unswitch is
        unspecified, it defaults to the same value as switch.

        @param traj: timed trajectory to execute
        @type  traj: openravepy.Trajectory
        @param defer: execute asynchronously and return a trajectory Future
        @type  defer: bool
        @param timeout: maximum time to wait for execution to finish
        @type  timeout: float or None
        @param switch: switch to the controllers necessary to execute traj
        @type  switch: bool
        @param unswitch: revert the controllers after executing traj
        @type  unswitch: bool or None
        @return trajectory executed on the robot
        @rtype  openravepy.Trajectory or TrajectoryFuture
        """
        from .util import or_to_ros_trajectory, pad_ros_trajectory
        from rospy import Time

        if self.simulated:
            return Robot.ExecuteTrajectory(self, traj, defer=defer, timeout=timeout,
                                           **kwargs)

        if unswitch is None:
            unswitch = switch

        traj_msg = or_to_ros_trajectory(self, traj)

        # The trajectory_controller expects the full set of DOFs to be
        # specified in every trajectory. We pad the trajectory by adding
        # constant values for any missing joints.
        with self.GetEnv():
            pad_ros_trajectory(self, traj_msg, self._trajectory_joint_names)

        if switch:
            self._trajectory_switcher.switch()

        traj_future = self._trajectory_client.execute(traj_msg)

        if unswitch:
            traj_future.add_done_callback(
                lambda _: self._trajectory_switcher.unswitch
            )

        if defer:
            return traj_future
        else:
            return traj_future.result(timeout)
        #return traj_future
