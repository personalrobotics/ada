import numpy
import prpy.tsr
from prpy.tsr.tsrlibrary import TSRFactory
from ada_block_sorting.settings import Settings

@TSRFactory('ada', 'block', 'grasp')
def block_grasp(robot, block, manip=None):
    """
    Generates end-effector poses for moving the arm near the block
    @param robot The robot grasping the block
    @param block The block being grasped
    @param manip The manipulator to move near the block, if None
       the active manipulator of the robot is used
    """
    #import IPython
    #IPython.embed()
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    block_in_world = block.GetTransform()
    '''
    For block 1
    [[ 1.       0.       0.      -0.3305 ]
     [ 0.       1.       0.      -0.08175]
     [ 0.       0.       1.      -0.0455 ]
     [ 0.       0.       0.       1.     ]]
    '''
    block_in_world[:3,:3] = numpy.eye(3) # ignore orientation
    '''
    [[ 1.       0.       0.      -0.3305 ]
     [ 0.       1.       0.      -0.08175]
     [ 0.       0.       1.      -0.0455 ]
     [ 0.       0.       0.       1.     ]]
    '''
    # we will transform the axie, we want to rotate along the block y axis for an angle alpha
    # then translate for a distance offset
    offset = Settings.BLOCK_GRASP_TSR_HEIGHT # the distance translated along z axis
    alpha = Settings.BLOCK_GRASP_TSR_ALPHA/180.*numpy.pi
    x_translate = offset * numpy.tan(alpha)
    y_translate = x_translate
    ee_in_block = numpy.array(
        [[numpy.cos(alpha),     0.,     numpy.sin(alpha),  0.],
        [ 0.,                   1.,     0,                 0.],
        [ -numpy.sin(alpha),    0.,     numpy.cos(alpha),  offset],
        [ 0.,                   0.,     0.,                1.]])
    '''
    Then we let the hand be parallel to the table
    Because ada and herb have different grasping techniques
    Herb
      1. go to the best sampled TSR configuration
          and at this time, his finger is about 45 degree above the horizon
      2. 2 fingers touch the table
      3. push
      4. 3rd finger closes to grasp the block
    Ada
      Here we are trying to do similar thing
      1. go to the best sampled TSR configuration
          and at this time, his finger is about 0 degree above the horizon - parallel to the table
      2. 2 fingers touch the table
      3. push
      4. 2 fingers close to grasp the block
    '''
    # this is boundary, it is a matrix to store the upper and lower bound of x,y,z,theta
    # x, y, z, roll, pitch, yaw
    Bw = numpy.zeros((6,2))
    Bw[5,:] = [-numpy.pi+.000, numpy.pi-.0001]
    # Bw[0,0] = -x_translate
    # Bw[0,1] = x_translate
    # Bw[1,0] = -y_translate
    # Bw[1,1] = y_translate

    '''
    [[ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [-3.14159265  3.14149265]]
    '''

    '''
    T0_w = block location in world frame
    Tw_e = ee location in block frame
    bw = allowed movement of ee in block frame
    '''
    pose_tsr = prpy.tsr.TSR(T0_w = block_in_world,
                             Tw_e = ee_in_block,
                             Bw = Bw,
                             manip = manip_idx)

    pose_tsr_chain = prpy.tsr.TSRChain(sample_start=False,
                                        sample_goal = True,
                                        TSRs = [pose_tsr])
    return [pose_tsr_chain]

@TSRFactory('ada', 'block', 'place')
def block_at_pose(robot, block, position, manip=None):
    '''
    Generates end-effector poses for placing the block on another object

    @param robot The robot grasping the block
    @param block The block being grasped
    @param position The position to place the block [x,y,z]
    @param manip The manipulator grasping the object, if None the
       active manipulator of the robot is used
    '''

    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    # Block on the object
    T0_w = numpy.eye(4)
    T0_w[:3,3] = position

    Bw = numpy.zeros((6,2))
    Bw[5,:] = [-numpy.pi, numpy.pi]

    place_tsr = prpy.tsr.TSR(T0_w = T0_w,
                             Tw_e = numpy.eye(4),
                             Bw = Bw,
                             manip = manip_idx)

    ee_in_block = numpy.dot(numpy.linalg.inv(block.GetTransform()), manip.GetEndEffectorTransform())
    ee_tsr = prpy.tsr.TSR(T0_w = numpy.eye(4), #ignored
                          Tw_e = ee_in_block,
                          Bw = numpy.zeros((6,2)),
                          manip = manip_idx)

    place_tsr_chain = prpy.tsr.TSRChain(sample_start=False,
                                        sample_goal = True,
                                        TSRs = [place_tsr, ee_tsr])
    return [place_tsr_chain]

@TSRFactory('ada', 'block', 'place_on')
def block_on_surface(robot, block, pose_tsr_chain, manip=None):
    '''
    Generates end-effector poses for placing the block on a surface.
    This factory assumes the block is grasped at the time it is called.

    @param robot The robot grasping the block
    @param block The grasped object
    @param pose_tsr_chain The tsr chain for sampling placement poses for the block
    @param manip The manipulator grasping the object, if None the active
       manipulator of the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
        manip = robot.GetActiveManipulator()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    '''
    T0_w = block location in world frame
    Tw_e = ee location in block frame
    bw = allowed movement of ee in block frame
    '''

    block_pose = block.GetTransform()
    block_pose[:3,:3] = numpy.eye(3) # ignore orientation
    ee_in_block = numpy.dot(numpy.linalg.inv(block_pose), manip.GetEndEffectorTransform())
    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0., 0.04]  # Allow some vertical movement

    for tsr in pose_tsr_chain.TSRs:
        if tsr.manipindex != manip_idx:
            raise Exception('pose_tsr_chain defined for a different manipulator.')

    grasp_tsr = prpy.tsr.TSR(Tw_e = ee_in_block, Bw = Bw, manip = manip_idx)
    all_tsrs = list(pose_tsr_chain.TSRs) + [grasp_tsr]
    place_chain = prpy.tsr.TSRChain(sample_start = False, sample_goal = True, constrain = False,
                           TSRs = all_tsrs)

    return  [ place_chain ]

