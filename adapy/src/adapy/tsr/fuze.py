import numpy
from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *

@TSRFactory('ada', 'fuze_bottle', 'lift')
def glass_lift(robot, bottle, manip=None, distance=0.1):
    '''
    This creates a TSR for lifting the glass a specified distance.  
    It assumed that when called, the robot is grasping the bottle.

    @param robot The robot performing the lift
    @param glass The bottle to lift
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    @param distance The distance to lift the glass
    '''

    print 'distance = %0.2f' % distance

    if manip is None:
        manip = robot.GetActiveManipulator()
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
         with manip.GetRobot():
             manip.SetActive()
             manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    #TSR for the goal
    start_position = manip.GetEndEffectorTransform()
    end_position = manip.GetEndEffectorTransform()
    end_position[2, 3] += distance

    Bw = numpy.zeros((6, 2))
    epsilon = 0.05
    Bw[0,:] = [-epsilon, epsilon]
    Bw[1,:] = [-epsilon, epsilon]
    Bw[4,:] = [-epsilon, epsilon]

    tsr_goal = TSR(T0_w = end_position, Tw_e = numpy.eye(4),
            Bw = Bw, manip = manip_idx)

    goal_tsr_chain = TSRChain(sample_start = False, sample_goal = True,
            constrain = False, TSRs = [tsr_goal])

    #TSR that constrains the movement
    Bw_constrain = numpy.zeros((6, 2))
    Bw_constrain[:, 0] = -epsilon
    Bw_constrain[:, 1] = epsilon
    if distance < 0:
        Bw_constrain[1,:] = [-epsilon+distance, epsilon]
    else:
        Bw_constrain[1,:] = [-epsilon, epsilon+distance]

    tsr_constraint = TSR(T0_w = start_position, Tw_e = numpy.eye(4),
            Bw = Bw_constrain, manip = manip_idx)

    movement_chain = TSRChain(sample_start = False, sample_goal = False,
            constrain = True, TSRs = [tsr_constraint])

    return [goal_tsr_chain, movement_chain]

@TSRFactory('ada', 'fuze_bottle', 'grasp')
def fuze_grasp(robot, fuze, manip=None):
    '''
    @param robot The robot performing the grasp
    @param glass The bottle to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''

    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = fuze.GetTransform()

    ee_to_palm = 0.15
    palm_to_bottle_center = 0.045
    total_offset = ee_to_palm + palm_to_bottle_center
    Tw_e = numpy.array([[ 0., 0., -1., -total_offset],
                        [ 0., 1.,  0., 0.],
                        [ 1., 0.,  0., 0.08], #glass_height
                        [ 0., 0.,  0., 1.]])


    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement
    Bw[5,:] = [-numpy.pi/2, numpy.pi/2]  # Allow any orientation

    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True,
                           constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
