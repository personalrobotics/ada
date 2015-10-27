import logging, numpy, openravepy, prpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from prpy.util import ComputeEnabledAABB

from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *
from adapy.tsr import block
from adapy.tsr import block_bin

logger = logging.getLogger('ada_block_sorting')

class NoTSRException(Exception):
    pass

def _GrabBlock(robot, blocks, table, manip=None, preshape=None,
              **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    @param preshape The shape (dof_values) to move the hand to before
      executing the grab
    """
    from prpy.rave import AllDisabled, Disabled
    from prpy.viz import RenderTSRList, RenderVector
    #print 'grabblock'
    env = robot.GetEnv()
    block = None

    if manip is None:
        with env:
            manip = robot.GetActiveManipulator()

    # 1. We first set up the pre shape of the hand
    # [0.0,0.0] - gripper is completely open to the maximum
    # [1.0,1.0] - gripper is completely closed and overlapped between 2 fingers
    # if the fingers is not correctly set up, it will generate an error, but that should be ok
    if preshape is None:
        manip.hand.MoveHand(f1=0.5,f2=0.5)
    else:
        # * is transposing the matrix
        manip.hand.MoveHand(*preshape)

    # 2. Now let's grab block
    # prpy/src/prpy/tsr/tsrlibrary.py
    # we need to copy the block.py and table.py from herbpy/src/herbpy/tsr to adapy/src/adapy/tsr


    # The TSR for ada should be different with the one for herb
    block_tsr_list = []

    for b in blocks:
        # Get a TSR to move near the block.
        # Here, adapy/tsr/block.py will be called. Because it is decorated by a TSRFactory class
        # The TSR calculation will be done in block.py
        tsr_list = robot.tsrlibrary(b, 'grasp', manip=manip)
        block_tsr_list += tsr_list
    # we should draw the axes of manipulator in the specific pose first before the planning
    # so that we can see clearly if we can plan the hand there or not
    # h = openravepy.misc.DrawAxes(env,manip.GetEndEffectorTransform())

    # Plan to a pose above the block
    # this will sample all the TSRlists for all the blocks, and choose the best one.
    # So this place is how the target block is selected.
    # prpy/src/prpy/planning/tsr.py
    with RenderTSRList(block_tsr_list, robot.GetEnv()):
        with Disabled(table, padding_only=True):
            # seems like they are same - manip.Plan or robot.Plan
            # traj = manip.PlanToTSR(block_tsr_list, execute=False)
            traj = robot.PlanToTSR(block_tsr_list, execute=False)
            # plan TSR is not timed, we have to use ExecutePath
            # robot.ExecuteTrajectory(traj) 
            robot.ExecutePath(traj)
    
    with manip.GetRobot().GetEnv():
        ee_pose = manip.GetEndEffectorTransform()

    block_ee_distance = [(tsr_chain.distance(ee_pose))[0] for tsr_chain in block_tsr_list]
    if len(block_ee_distance) == 0:
        raise NoTSRException("Failed to find the TSR PlanToTSR planned to")
    min_index = numpy.argmax(block_ee_distance)

    # in real robot, if any error, then the robot may not in the tsr, so we cannot use the 'contain'
    # block_idxs = [ idx for idx, tsr_chain in enumerate(block_tsr_list)
    #               if tsr_chain.contains(ee_pose) ]
    # if len(block_idxs) == 0:
    #     raise NoTSRException("Failed to find the TSR PlanToTSR planned to")
    # block = blocks[block_idxs[0]]

    '''

[[  5.01101931e-01   8.65388157e-01  -4.39701454e-04   6.68751162e-01]
 [ -8.65388258e-01   5.01101937e-01  -1.02481789e-04   6.72092399e-01]
 [  1.31648724e-04   4.31866298e-04   9.99999898e-01   1.09949273e+00]
 [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]

    '''
    
    block = blocks[min_index]

    # h = openravepy.misc.DrawAxes(env,manip.GetEndEffectorTransform())

    try:
        with AllDisabled(env, [table] + blocks, padding_only=True):
            with env:
                # First we should close the finger to a certain degree
                # that it will not collide with other blocks on the table
                preshape_finger = 0.5
                robot.arm.hand.MoveHand(preshape_finger,preshape_finger)

                # Then we move the hand down to the table
                # this is just a translation, so we just need the 4th column in GetEndEffectorTransform
                #   as the start_point
                table_aabb = ComputeEnabledAABB(table)
                table_height = abs(table_aabb.pos()[2] + table_aabb.extents()[2])
                # 0.14 is the distance from finger tip to end-effector frame
                # [2,3] is the z of finger
                current_finger_height = manip.GetEndEffectorTransform()[2,3] - 0.14

                start_point = manip.GetEndEffectorTransform()[0:3, 3]
                to_block_direction = block.GetTransform()[:3,3] - manip.GetEndEffectorTransform()[:3,3]

            '''
            RenderVector - prpy/src/prpy/viz.py
            Render a vector in an openrave environment
            @param start_pt The start point of the vector
            @param direction The direction of the vector to render
            @param length The length of the rendered vector
            '''
            min_distance = current_finger_height - table_height
            # for testing
            min_distance = 0.14
            with RenderVector(start_point, to_block_direction, min_distance, env):
                # goal_point = start_point + to_block_direction * min_distance
                # goal_transform = manip.GetEndEffectorTransform()
                # goal_transform[0:3,3] = goal_point
                # manip.PlanToEndEffectorPose(goal_transform,execute=True);
                manip.PlanToEndEffectorOffset(direction=to_block_direction,
                        distance=min_distance, max_distance=min_distance+0.05,
                        timelimit=5., execute=True)

        # Close the finger to grab the block
        manip.hand.MoveHand(f1=1.,f2=1.)
        # Compute the pose of the block in the hand frame
        with env:
            # local_p = [0.01, 0, 0.24, 1.0]
            hand_pose = manip.GetEndEffectorTransform()
            # world_p = numpy.dot(hand_pose, local_p)
            block_pose = block.GetTransform()
            # block_pose[:,3] = world_p
            block_relative = numpy.dot(numpy.linalg.inv(hand_pose), block_pose)

        # Now lift the block up off the table
        with AllDisabled(env, blocks + [table]):
            manip.PlanToEndEffectorOffset(direction=[0, 0, 1], distance=0.05,
                                          timelimit=5, execute=True)

        # OpenRAVE trick to hallucinate the block into the correct pose relative to the hand
        with env:
            hand_pose = manip.GetEndEffectorTransform()
            block_pose = numpy.dot(hand_pose, block_relative)
            block.SetTransform(block_pose)
            manip.GetRobot().Grab(block)

    except PlanningError as e:
        logger.error('Failed to complete block grasp')
        raise
    finally:
        return block


'''
function decorator
http://thecodeship.com/patterns/guide-to-python-function-decorators/
Here, GrabBlocks is wrapped by the wrapper ActionMethod
(which is in prpy/src/prpy/action/actionlibrary.py)
In actionlibrary.py which is inherited from 'object'
def __init__(self, func) - the func argument <= GrabBlocks function


'''
@ActionMethod
def GrabBlocks(robot, blocks, table, **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    """
    return _GrabBlock(robot, blocks, table, **kw_args)

@ActionMethod
def GrabBlock(robot, block, table, **kw_args):
    """
    @param robot The robot performing the grasp
    @param block The block to grab
    @param table The table, or object, the block is resting on
    @param manip The manipulator to use to grab the block
      (if None active manipulator is used)
    """
    return _GrabBlock(robot, [block], table, **kw_args)

@ActionMethod
def PlaceBlock(robot, block, on_obj, manip=None, **kw_args):
    """
    Place a block on an object. This function assumes the block
    is grabbed by the robot when the method is called.
    @param robot The robot performing the block placement
    @param block The block to be placed
    @param on_obj The object to place the block on, this object
    must have a 'point_tsr defined
    @param manip The manipulator used to perform the placement
    If none, the active manipulator is used.
    """
    env = robot.GetEnv()
    # Get a tsr for this position
    # Here, adapy/tsr/block.py will be called. Because it is decorated by a TSRFactory class
    # The TSR calculation will be done in block.py

    # I need to copy that file to adapy
    # take the block to the top of the bin
    '''
    (1) calculate the target TSR above the bin's BASE.
        if we just plan to object_place_list and get rid of place_tsr_list,
        then the hand will go to the deep inside the bin near the bottom
        why???????????????????????????????/
    '''
    # 'herb', 'block_bin', 'point_on' TSR is in herbpy/src/herbpy/tsr/block_bin.py
    object_place_list = robot.tsrlibrary(on_obj, 'point_on', manip=manip)

    '''
    (2) place_tsr_list - the current TSR before taking the block to the bin.
          So this TSR is a stable posture
          and it is equal to the posture where the hand is about to release the block.
          So we connect object_place_list with place_tsr_list into one TSR list,
          so that the hand will move toward the bottom inside the bin and after it got to
          the goal TSR position, it will open the finger and release the block.
          It will also not move to the deep inside in the bin
    '''
    # 'herb', 'block_bin', 'place_on' TSR is in  herbpy/src/herbpy/tsr/block.py
    from adapy.tsr.block import block_grasp
    from adapy.tsr.block import block_at_pose
    from adapy.tsr.block import block_on_surface

    place_tsr_list = robot.tsrlibrary(block, 'place_on',
        pose_tsr_chain=object_place_list[0], manip=manip)

    # Plan there
    with prpy.viz.RenderTSRList(object_place_list, robot.GetEnv()):
        manip.PlanToTSR(place_tsr_list, execute=True)

    # Open the hand and drop the block
    manip.hand.MoveHand(f1=0.2,f2=0.2)

    with env:
        # if we don't do this, then the block will stick to the hand.
        # even we try to drop the block, it cannot move!!!
        # we have to switch their status to release!
        # where is this function???????????????????/
        manip.GetRobot().Release(block)

        # Move the block down until it hits something
        block_pose = block.GetTransform()
        # we move the block down in viewer by our hand...
        while not env.CheckCollision(block) and block_pose[2,3] > 0.0:
            block_pose[2,3] -= 0.02
            block.SetTransform(block_pose)

