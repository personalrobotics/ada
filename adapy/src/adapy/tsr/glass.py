import numpy

from prpy.tsr.tsrlibrary import TSRFactory
from prpy.tsr.tsr import *
@TSRFactory('ada', 'glass', 'move')
def move_glass(robot, glass, manip=None):
    '''
    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = glass.GetTransform()
    Tw_e = numpy.array([[ 0., 0., -1., -0.22], 
                          [-1., 0., 0., 0.1], 
                          [0., 1., 0., 0.28], 
                          [0., 0., 0., 1.]])
    #Tw_e = numpy.array([[ 1., 0., 0., 0.], 
    #                     [0., 1., 0., 0.], 
    #                     [0., 0., 1., 0.], 
    #                     [0., 0., 0., 1.]])

    Bw = numpy.zeros((6,2))
    Bw[2,:] = [-0.02, 0.02]  # Allow a little vertical movement
    #Bw[5,:] = [-numpy.pi, numpy.pi]  # Allow any orientation
    
    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, constrain=False, TSR = grasp_tsr)

    return [grasp_chain]


@TSRFactory('ada', 'glass', 'grasp')
def glass_grasp(robot, glass, manip=None):
    '''
    @param robot The robot performing the grasp
    @param glass The glass to grasp
    @param manip The manipulator to perform the grasp, if None
       the active manipulator on the robot is used
    '''
    if manip is None:
        manip_idx = robot.GetActiveManipulatorIndex()
    else:
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

    T0_w = glass.GetTransform()
    #Tw_e = numpy.array([[ 0., 0., 1., -0.225], 
    #                      [1., 0., 0., 0.], 
    #                      [0., 1., 0., 0.08], 
    #                      [0., 0., 0., 1.]])
    Tw_e = numpy.array([[ 0., 0., -1., -0.02], 
                          [-1., 0., 0., 0.], 
                          [0., 1., 0., 0.08], 
                          [0., 0., 0., 1.]])


    Bw = numpy.zeros((6,2))
    Bw[2,:] = [0.0, 0.02]  # Allow a little vertical movement

    #Bw[2,:] = [-0.001, 0.001]  # Allow a little vertical movement

    #Bw[5,:] = [-numpy.pi/2, numpy.pi/2]  # Allow any orientation
    #Bw[5,:] = [0, numpy.pi/2]  # Allow any orientation
    #Bw[5.:] = [numpy.pi -numpy.pi/6, numpy.pi + numpy.pi/2]
    #Bw[5.:] = [numpy.pi + numpy.pi/2 + numpy.pi/3, numpy.pi + numpy.pi/2 + 2*numpy.pi/3]
    Bw[5.:] = [numpy.pi - numpy.pi/2, numpy.pi]
    #Bw[5,:] = [numpy.pi, numpy.pi + numpy.pi]# works for the glass demo

    
    grasp_tsr = TSR(T0_w = T0_w, Tw_e = Tw_e, Bw = Bw, manip = manip_idx)
    grasp_chain = TSRChain(sample_start=False, sample_goal = True, constrain=False, TSR = grasp_tsr)

    return [grasp_chain]
