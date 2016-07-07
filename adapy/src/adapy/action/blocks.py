import logging, numpy, openravepy, prpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from prpy.util import ComputeEnabledAABB
from block_description_generator_networkx.spatial_description_generator import spatial_desc_gen

from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *
from adapy.tsr import block
from adapy.tsr import block_bin

logger = logging.getLogger('ada_block_sorting')

move_to_table_min_distance = 0.07
move_to_table_max_distance = None


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
    # we need to copy the block.py and table.py from adapy/src/adapy/tsr to adapy/src/adapy/tsr


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

    # we should use this to only pick up the blocks which is very close to the ee_pose,
    # but because of error in real machine, it is very possible that all of our blocks are larger than the threshold distance
    # block_idxs = [ idx for idx, tsr_chain in enumerate(block_tsr_list)
    #               if tsr_chain.contains(ee_pose) ]
    # if len(block_idxs) == 0:
    #     raise NoTSRException("Failed to find the TSR PlanToTSR planned to")
    # block = blocks[block_idxs[0]]
    # ==>
    # therefore, we will instead rank the distance and choose the block with a minimal distance
    dist_list = [tsr_chain.distance(ee_pose)[0] for idx, tsr_chain in enumerate(block_tsr_list)]   
    if len(dist_list) == 0:
        raise NoTSRException("Failed to find the TSR PlanToTSR planned to")
    block = blocks[numpy.argmin(dist_list)]


    # h = openravepy.misc.DrawAxes(env,manip.GetEndEffectorTransform())
    '''
    # ---------------------------------------------------------
    # Shen Li
    # 1. AABB - bounding box
    #   AABB includes the information about transformation matrix (x,y of the CoM)
    #   (bb.GetConfigurationValues() or bb.GetTransform())
    # 2. name
    # 3. color
    # import IPython;
    # IPython.embed()


    import yaml
    import ada_block_sorting.convert_color as closest_color
    block_data = {}
    for bb in blocks:
        block_name = bb.GetName() # string: b1, b2, ...
        block_data[block_name] = {}
        block_data[block_name]['name'] = str(block_name)

        # Get the color of the block
        geom = bb.GetLinks()[0].GetGeometries()[0]
        color = geom.GetDiffuseColor()
        # Re-scale the block's RGB color from float to 0-255
        block_color = (color[0]*255.0, color[1]*255.0, color[2]*255.0)
        # Get the English color name that most closely matches the RGB value
        actual_color_name, closest_color_name = closest_color.get_colour_name(block_color)
        block_data[block_name]['color_RGB'] = block_color
        block_data[block_name]['color_name'] = str(closest_color_name)

        bounding_box = bb.ComputeAABB()

        # we need to reverse x with y because now the y is actually horizontal
        # for example, in this example, the robot is closest to the yellow one,
        # so we have to rotate the table to make it consistent
        block_data[block_name]['AABB_pos'] = [bounding_box.pos()[1], bounding_box.pos()[0]]
        block_data[block_name]['AABB_ext'] = [bounding_box.extents()[1], bounding_box.extents()[0]]

    table_AABB_pos = [table.ComputeAABB().pos()[1], table.ComputeAABB().pos()[0]]
    table_AABB_ext = [table.ComputeAABB().extents()[1], table.ComputeAABB().extents()[0]]
    # for key, value in block_data.iteritems():
        # print value
    # with open('./block_scenario.yml', 'w') as outfile:
        # outfile.write( yaml.dump(block_data, default_flow_style=False) )
    target_block_name = block.GetName()
    num_of_solu_needed = 1
    print 'spatial_desc_gen starts-----------------------------'

    desc_list = spatial_desc_gen(block_data, str(target_block_name),\
        table_AABB_pos, table_AABB_ext, False, num_of_solu_needed)

    if not desc_list:
        desc_list = []
        desc_list[0] = 'Sorry, I could not describe this block I am going to pick up.'
    else:
        print 'demo.py: Spatial description of this block:'
        for i in xrange(len(desc_list)):
            print 'Solution '+str(i+1)+' : '+str(desc_list[i])

        best_solution = desc_list[0]
        print 'best_solution='+ best_solution
        print 'spatial_desc_gen ends-----------------------------'

    import IPython;IPython.embed()
    robot.Say(desc_list[0])
    import IPython;IPython.embed()

    # ---------------------------------------------------------
    '''
    
    # try:
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
            current_finger_height = manip.GetEndEffectorTransform()[2,3]-0.14
            block_height = block.GetTransform()[2,3]
        
        # manip.GetEndEffectorTransform()
        # [[ -1.26761961e-02   4.75773380e-01  -8.79476552e-01   4.53303877e-02]
        #  [  6.81338158e-03   8.79567899e-01   4.75724593e-01   8.97872675e-02]
        #  [  9.99896441e-01   3.81689084e-05  -1.43912008e-02   3.14018485e-01]
        #  [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
        
        # h = openravepy.misc.DrawAxes(env,tran)

        # RenderVector - prpy/src/prpy/viz.py
        # Render a vector in an openrave environment
        # @param start_pt The start point of the vector
        # @param direction The direction of the vector to render
        # @param length The length of the rendered vector
        # 0.16
        min_distance = current_finger_height - table_height
        down_direction = [0., 0., -1.]

        with AllDisabled(env, blocks + [table]):
            with RenderVector(start_point, down_direction, min_distance, env):
                # https://github.com/personalrobotics/prpy/blob/ecbf890d9e4f8e57616c049b0edae8390ee2c4c8/src/prpy/planning/workspace.py
                # Plan to a desired end-effector offset with move-hand-straight
                # constraint. movement less than distance will return failure.
                # The motion will not move further than max_distance.
                manip.PlanToEndEffectorOffset(direction=down_direction,
                    distance=move_to_table_min_distance, max_distance=move_to_table_max_distance,
                    timelimit=5., execute=True)


        # Policy 1 Close the finger to grab the block
        # manip.hand.MoveHand(f1=1.,f2=1.)
        # Policy 2 Create class object of openLoopGrasper with optimized parameters
        from openLoopGrasper import openLoopGrasper
        O = openLoopGrasper(robot,env)
        (success,trajLength) = O.PolicyExecute(1.85,1,10,3)
        # (success,trajLength) = O.PolicyExecute(4.77,1,9.55,7)
        print success,trajLength

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

    # except PlanningError as e:
    #     logger.error('Failed to complete block grasp')
    #     raise
    # finally:
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
    # 'ada', 'block_bin', 'point_on' TSR is in adapy/src/adapy/tsr/block_bin.py
    object_place_list = robot.tsrlibrary(on_obj, 'point_on', manip=manip)

    '''
    (2) place_tsr_list
          So this TSR is a stable posture, just varify EE's vertical position
          and it is equal to the posture where the hand is about to release the block.
          So we connect object_place_list with place_tsr_list into one TSR list,
          so that the hand will move toward the bottom inside the bin and after it got to
          the goal TSR position, it will open the finger and release the block.
          It will also not move to the deep inside in the bin
    '''
    # 'ada', 'block_bin', 'place_on' TSR is in  adapy/src/adapy/tsr/block.py
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
