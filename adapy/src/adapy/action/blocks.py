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
    # print 'grabblock'
    env = robot.GetEnv()
    block = None

    if manip is None:
        with env:
            manip = robot.GetActiveManipulator()

    # 1. We first set up the pre shape of the hand
    # [0.0,0.0] - gripper is completely open to the maximum
    # [1.0,1.0] - gripper is completely closed and overlapped between 2 fingers
    if preshape is None:
        preshape=[0.5,0.5]
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
    # prpy/src/prpy/planning/tsr.py
    with RenderTSRList(block_tsr_list, robot.GetEnv()):
        with Disabled(table, padding_only=True):
            manip.PlanToTSR(block_tsr_list, execute=True)
    with manip.GetRobot().GetEnv():
        ee_pose = manip.GetEndEffectorTransform()

    block_idxs = [ idx for idx, tsr_chain in enumerate(block_tsr_list)
                  if tsr_chain.contains(ee_pose) ]
    if len(block_idxs) == 0:
        raise NoTSRException("Failed to find the TSR PlanToTSR planned to")
    
    block = blocks[block_idxs[0]]

    # h = openravepy.misc.DrawAxes(env,manip.GetEndEffectorTransform())

    try:
        with AllDisabled(env, [table] + blocks, padding_only=True):
            # Move down until touching the table
            with env:
                # this is just a translation, so we just need the 4th column in GetEndEffectorTransform
                #   as the start_point
                start_point = manip.GetEndEffectorTransform()[0:3, 3]
                table_aabb = ComputeEnabledAABB(table)
                table_height = table_aabb.pos()[2] + table_aabb.extents()[2]
                # 0.14 is the distance from finger tip to end-effector frame
                # [2,3] is the z of finger
                current_finger_height = manip.GetEndEffectorTransform()[2,3] - 0.14

            '''
            manip.GetEndEffectorTransform()
            [[ -1.26761961e-02   4.75773380e-01  -8.79476552e-01   4.53303877e-02]
             [  6.81338158e-03   8.79567899e-01   4.75724593e-01   8.97872675e-02]
             [  9.99896441e-01   3.81689084e-05  -1.43912008e-02   3.14018485e-01]
             [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
            '''
            # h = openravepy.misc.DrawAxes(env,tran)

            '''
            RenderVector - prpy/src/prpy/viz.py
            Render a vector in an openrave environment
            @param start_pt The start point of the vector
            @param direction The direction of the vector to render
            @param length The length of the rendered vector

            This MoveUntilTouch is from prpy.base import wam
            '''
            '''
            # (from blocksort in herbpy)
            # MoveUntilTouch is in wam, which is herb robot. There is no mico robot version of MoveUntilTouch
            # We need to write a new function in prpy/src/prpy/base/mico.py
            with RenderVector(start_point, down_direction, min_distance, env):
                manip.MoveUntilTouch(direction=down_direction, timelimit=5,
                    distance=min_distance, max_distance=min_distance + 0.05,
                    ignore_collisions=blocks + [table])
            '''
            min_distance = 0.10# current_finger_height - table_height
            down_direction = [0., 0., -1.]
            with RenderVector(start_point, down_direction, min_distance, env):
                manip.MoveUntilTouch(direction=down_direction, timelimit=5,
                    distance=min_distance, max_distance=min_distance + 0.05,
                    ignore_collisions=blocks + [table])

            # Move parallel to the table to funnel the block into the fingers
            # by projecting the -x direction of end-effector onto the xy-plane.
            #   the direction where the hand should move is to move forward
            #   so the direction = [x-hand,y-hand,0]

            with env:
                start_point = manip.GetEndEffectorTransform()[0:3, 3]
                '''
                manip.GetEndEffectorTransform()
                [[ -1.12592811e-02  -6.41083415e-01  -7.67388613e-01   1.75977270e-01]
                 [ -9.14263800e-03   7.67471181e-01  -6.41018251e-01  -2.00668205e-01]
                 [  9.99894815e-01  -2.01448366e-04  -1.45023673e-02   9.44411969e-02]
                 [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
                '''
                # start_point = [ 0.17597727 -0.20066821  0.0944412 ] - x y z of hand
                funnel_direction = -1. * manip.GetEndEffectorTransform()[:3,0]
                # funnel_direction = [ 0.01125928  0.00914264 -0.99989481] -
                funnel_direction[2] = 0.
                # funnel_direction = [ 0.01125928  0.00914264  0.        ]

            # TODO: We should only have to disable the block for this. Why does
            # this fail if we do not disable the table?
            # from openravepy import CollisionReport
            with AllDisabled(env, blocks + [table]):
                with RenderVector(start_point, funnel_direction, 0.1, env):
                    from prpy.planning.retimer import HauserParabolicSmoother
                    robot.retimer=HauserParabolicSmoother()
                    manip.PlanToEndEffectorOffset(direction=funnel_direction,
                        distance=0.08, max_distance=0.12,
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

