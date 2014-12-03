import logging, numpy, time, openravepy
import util

class MicoHand(openravepy.Robot.Link):
    def GetIndices(hand):
        return numpy.array(sorted(hand.manipulator.GetChildDOFIndices()))

    def GetDOFValues(hand):
        return hand.robot.GetDOFValues(hand.GetIndices())

    def SetDOFValues(hand, dof_values,
                     limits_action=openravepy.KinBody.CheckLimitsAction.CheckLimits):
        hand.robot.SetDOFValues(dof_values, hand.GetIndices(), limits_action)

    def LookAt(hand, **kw_args):
        target = hand.GetTransform()[0:3, 3]
        return hand.robot.LookAt(target, **kw_args)

    def MoveHand(hand, f1=None, f2=None, f3=None, spread=None, timeout=None):
        """
        Change the hand preshape. This function blocks until trajectory execution
        finishes. This can be changed by changing the timeout parameter to a
        maximum number of seconds. Pass zero to return instantantly.
        @param f1 finger 1 angle
        @param f2 finger 2 angle
        @param f3 finger 3 angle
        @param spread spread angle
        @param timeout blocking execution timeout
        """
        # Default any None's to the current DOF values.
        preshape = hand.GetDOFValues()
        if f1     is not None: preshape[0] = f1
        if f2     is not None: preshape[1] = f2
        if spread is not None: preshape[3] = spread


        hand.controller.SetDesired(preshape)
        util.WaitForControllers([ hand.controller ], timeout=timeout) 
       
    def OpenHand(hand, spread=None, timeout=None):
        """
        Open the hand with a fixed spread.
        @param spread hand spread
        @param timeout blocking execution timeout
        """
        if hand.simulated:
            robot = hand.GetParent()
            p = openravepy.KinBody.SaveParameters

            with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
                hand.manipulator.SetActive()
                robot.task_manipulation.ReleaseFingers()

            util.WaitForControllers([ hand.controller ], timeout=timeout)
        else:
            # TODO: Load this angle from somewhere.
            hand.MoveHand(f1=0.0, f2=0.0, spread=spread, timeout=timeout)

    def CloseHand(hand, spread=None, timeout=None):
        """
        Close the hand with a fixed spread.
        @param spread hand spread
        @param timeout blocking execution timeout
        """
        if hand.simulated:
            robot = hand.GetParent()
            p = openravepy.KinBody.SaveParameters

            with robot.CreateRobotStateSaver(p.ActiveDOF | p.ActiveManipulator):
                hand.manipulator.SetActive()
                robot.task_manipulation.CloseFingers()

            util.WaitForControllers([ hand.controller ], timeout=timeout)
        else:
            # TODO: Load this angle from somewhere.
            hand.MoveHand(f1=3.2, f2=3.2, spread=spread, timeout=timeout)

