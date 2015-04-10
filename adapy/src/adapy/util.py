import logging
from prpy.exceptions import PrPyException

logger = logging.getLogger('adapy.util')


class AdaPyException(PrPyException):
    pass


def find_adapy_resource(relative_path, package='adapy'):
    from catkin.find_in_workspaces import find_in_workspaces

    paths = find_in_workspaces(project=package, search_dirs=['share'],
                               path=relative_path, first_match_only=True)

    if paths and len(paths) == 1:
        return paths[0]
    else:
        raise IOError('Loading AdaPy resource "{:s}" failed.'.format(
                      relative_path))


def or_to_ros_trajectory(robot, traj):
    """ Convert an OpenRAVE trajectory to a ROS trajectory.

    @param robot: OpenRAVE robot
    @type  robot: openravepy.Robot
    @param traj: input trajectory
    @type  traj: openravepy.Trajectory
    """
    import numpy
    from rospy import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    if traj.GetEnv() != robot.GetEnv():
        raise ValueError(
            'Robot and trajectory are not in the same environment.')

    cspec = traj.GetConfigurationSpecification()
    dof_indices, _ = cspec.ExtractUsedIndices(robot)
    time_from_start = 0.

    traj_msg = JointTrajectory(
        joint_names=[ robot.GetJointFromDOFIndex(dof_index).GetName()
                      for dof_index in dof_indices ]
    )
    
    for iwaypoint in xrange(traj.GetNumWaypoints()):
        waypoint = traj.GetWaypoint(iwaypoint)

        dt = cspec.ExtractDeltaTime(waypoint)
        q = cspec.ExtractJointValues(waypoint, robot, dof_indices, 0)
        qd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 1)
        qdd = cspec.ExtractJointValues(waypoint, robot, dof_indices, 2)

        if dt == 0. and iwaypoint != 0:
            logger.warning('Skipped waypoint %d because deltatime = 0.',
                           iwaypoint)
            continue

        if dt is None:
            raise ValueError('Trajectory is not timed.')
        elif q is None:
            raise ValueError('Trajectory does not contain joint values')
        elif qdd is not None and qd is None:
            raise ValueError('Trajectory contains accelerations,'
                             ' but not velocities.')

        time_from_start += dt
        traj_msg.points.append(
            JointTrajectoryPoint(
                positions=q,
                velocities=qd if qd is not None else [],
                accelerations=qdd if qdd is not None else [],
                time_from_start=Duration.from_sec(time_from_start)
            )
        )

    assert numpy.isclose(time_from_start, traj.GetDuration())

    return traj_msg
