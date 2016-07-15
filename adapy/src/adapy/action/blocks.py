import logging, numpy, openravepy, prpy
from prpy.action import ActionMethod
from prpy.planning.base import PlanningError
from prpy.util import ComputeEnabledAABB
from block_description_generator_networkx.spatial_description_generator import spatial_desc_gen

from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *
from adapy.tsr import block
from adapy.tsr import block_bin

from scipy import signal



import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState

logger = logging.getLogger('ada_block_sorting')

reaching_table_speed = 0.02
move_to_table_min_distance = 0.01
move_to_table_max_distance = None

joint_1_ptp = 0.01
joint_2_ptp = 0.04
joint_3_ptp = 0.06


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

        block_data[block_name]['AABB_pos'] = [bounding_box.pos()[0], bounding_box.pos()[1]]
        block_data[block_name]['AABB_ext'] = [-1.*bounding_box.extents()[0], -1.*bounding_box.extents()[1]]

    # need to mirror the scenario to make the graph same as the simulation
    table_AABB_pos = [table.ComputeAABB().pos()[0], table.ComputeAABB().pos()[1]]
    table_AABB_ext = [-1.*table.ComputeAABB().extents()[0], -1.*table.ComputeAABB().extents()[1]]
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

    robot.Say(desc_list[0])

    # ---------------------------------------------------------
    
    
    # try:
    with AllDisabled(env, [table] + blocks, padding_only=True):
        with env:
            start_point = manip.GetEndEffectorTransform()
        
        down_direction = [0., 0., -1.]
        up_direction = [0., 0., 1.]

        with AllDisabled(env, blocks + [table]):
            # https://github.com/personalrobotics/prpy/blob/ecbf890d9e4f8e57616c049b0edae8390ee2c4c8/src/prpy/planning/workspace.py
            # Plan to a desired end-effector offset with move-hand-straight
            # constraint. movement less than distance will return failure.
            # The motion will not move further than max_distance.
            table_reached = False

            # ------------------------------------------------------------------
            rawData = numpy.array([])
            filteredData = numpy.array([])
            bufferMaxSize = 100
            b, a = signal.butter(4, 0.5, 'low', analog=False)     

            while table_reached == False:
                data = rospy.wait_for_message("/joint_states", JointState)
                # we just need the joint efforts from joint 1, 2, 3
                if rawData.size == 0:
                    rawData = numpy.array([data.effort[1], data.effort[2], data.effort[3]])
                else:
                    if len(rawData) < bufferMaxSize:
                        rawData = numpy.vstack([rawData, numpy.array([data.effort[1], data.effort[2], data.effort[3]])])
                    else:
                        rawData = numpy.vstack([rawData[1:], numpy.array([data.effort[1], data.effort[2], data.effort[3]])])
                    assert(len(rawData) <= bufferMaxSize)
                    print len(rawData)

                    # if len(rawData) >= bufferMaxSize:
                    if len(rawData) > 1:
                        filteredData = signal.lfilter(b, a, rawData)
                        # peak to peak
                        print filteredData[:,0].max(), filteredData[:,0].min(), filteredData[:,0].ptp()
                        print filteredData[:,1].max(), filteredData[:,1].min(), filteredData[:,1].ptp()
                        print filteredData[:,2].max(), filteredData[:,2].min(), filteredData[:,2].ptp()
                        if filteredData[:,0].ptp() >= joint_1_ptp or filteredData[:,1].ptp() >= joint_2_ptp or filteredData[:,2].ptp() >= joint_3_ptp:
                            print "Efforts gatherer: Efforts exceeding limits"
                            table_reached = True
                        else:
                            manip.PlanToEndEffectorOffset(direction=down_direction,
                                distance=reaching_table_speed, max_distance=move_to_table_max_distance,
                                timelimit=5., execute=True)
            manip.PlanToEndEffectorOffset(direction=up_direction,
                distance=move_to_table_min_distance, max_distance=move_to_table_max_distance,
                timelimit=5., execute=True)

            # cur_height = manip.GetEndEffectorTransform()[2,3]
            # start_point[2,3] = cur_height 
            # import IPython;IPython.embed()
            # robot.PlanToEndEffectorPose(start_point, execute=True)

            # ------------------------------------------------------------------


        # Policy 1 Close the finger to grab the block
        # manip.hand.MoveHand(f1=1.34,f2=1.34)
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
