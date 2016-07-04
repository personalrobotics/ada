#!/usr/bin/env python
import adapy, openravepy, numpy, prpy, rospy
from IPython import embed
import numpy as np
import copy
from std_msgs.msg import Time
from std_msgs.msg import Bool

def PlanToTransform(env, robot, transform):
    handle = openravepy.misc.DrawAxes(env, transform);
    iksolver = robot.arm.GetIkSolver()
    param = openravepy.IkParameterization(transform, openravepy.IkParameterizationType.Transform6D)
    solution = iksolver.Solve(param, robot.GetActiveDOFValues(),  0)
    traj =  robot.PlanToConfiguration(solution.GetSolution())
    return traj;

def PlanToOffset(env, robot, offset):
    transform = robot.arm.GetEndEffectorTransform()
    transform[0:3, 3] += offset;
    traj = PlanToTransform(env, robot, transform);
    return traj

def wp(c, **kwargs):
   v = [0.]*c.GetDOF()
   for gname,gvals in kwargs.items():
      g = c.GetGroupFromName(gname)
      if g.dof != len(gvals):
         raise RuntimeError('vals length mismatch!')
      for i in range(g.dof):
         v[i+g.offset] = gvals[i]
   return v

def RotPolicyToTraj(alpha,deltatime,env,robot):
	d_rot = 0.1
	maxInd = 100000
	robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
	c = robot.GetActiveConfigurationSpecification(interpolation='linear')
	c.AddDeltaTimeGroup()
	traj = openravepy.RaveCreateTrajectory(env, '')
	traj.Init(c)
	initConfig = robot.GetActiveDOFValues()
	initConfig = np.hstack((initConfig,[deltatime]))
	d_policy = np.array([-alpha*d_rot,-alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
	currConfig = copy.deepcopy(initConfig)
	currInd = 0
	traj.Insert(currInd,currConfig)
	while currInd<=maxInd and currConfig[0]<0.9:
		currConfig = currConfig - d_policy
		print currConfig
		currInd = currInd + 1
		traj.Insert(currInd,currConfig)
	return traj

	maxInd = 10000
def PolicyExecute(alpha,deltatime,backFactor,NoI,env,robot):
	robot.arm.hand.MoveHand(0.2,0.2)
	traj = FreqPolicyToTraj(alpha,deltatime,backFactor,NoI,env,robot)
	robot.ExecuteTrajectory(traj)
	rospy.sleep(1)
	robot.arm.hand.MoveHand(1.2,1.2)

def FreqPolicyToTraj(alpha,deltatime,backFactor,NoI,env,robot):
	d_rot = 0.1
	minConfig = 1.1

	robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
	c = robot.GetActiveConfigurationSpecification(interpolation='linear')
	c.AddDeltaTimeGroup()
	traj = openravepy.RaveCreateTrajectory(env, '')
	traj.Init(c)
	initConfig = robot.GetActiveDOFValues()
	initConfig = np.hstack((initConfig,[deltatime]))
	maxConfig = initConfig[0]
	dFinger = minConfig-maxConfig
	d_policy = np.array([-alpha*d_rot,-alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
	currConfig = copy.deepcopy(initConfig)
	currInd = 0
	traj.Insert(currInd,currConfig)
	for closeLooper in range(0,NoI):
		d_policy = np.array([-alpha*d_rot,-alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
		while currConfig[0]<minConfig:
			currConfig = currConfig - d_policy
			print currConfig
			currInd = currInd + 1
			traj.Insert(currInd,currConfig)
		d_policy = np.array([backFactor*alpha*d_rot,backFactor*alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
		while currConfig[0]>maxConfig:
			currConfig = currConfig - d_policy
			print currConfig
			currInd = currInd + 1
			traj.Insert(currInd,currConfig)
	return traj

	maxInd = 10000

def OpenCloseRep(N,tgap,robot):
	pub_open = rospy.Publisher('gripper_open', Time, queue_size=1)
	pub_close = rospy.Publisher('gripper_close', Time, queue_size=1)
	for looper in range(0,N):
		robot.arm.hand.MoveHand(0.1,0.1)
		rospy.sleep(tgap)
		pub_open.publish(Time(rospy.Time.now()))
		robot.arm.hand.MoveHand(1.34,1.34)
		rospy.sleep(tgap)
		pub_close.publish(Time(rospy.Time.now()))
		robot.arm.hand.MoveHand(0.1,0.1)

		
def OpenCloseTandemRep(N,tgap,robot):
	pub_open = rospy.Publisher('gripper_open', Time, queue_size=1)
        pub_close = rospy.Publisher('gripper_close', Time, queue_size=1)
        for looper in range(0,N):
                robot.arm.hand.MoveHand(0.1,0.1)
                rospy.sleep(tgap)
                pub_open.publish(Time(rospy.Time.now()))
                robot.arm.hand.MoveHand(1.34,0.1)
		rospy.sleep(1)
                robot.arm.hand.MoveHand(1.34,1.34)
                rospy.sleep(tgap)
                pub_close.publish(Time(rospy.Time.now()))
                robot.arm.hand.MoveHand(0.1,0.1)

def CloseTandem(tgap,robot):
        pub_close = rospy.Publisher('gripper_close', Time, queue_size=1)
        robot.arm.hand.MoveHand(1.34,0.1)
	rospy.sleep(1)
        robot.arm.hand.MoveHand(1.34,1.34)
        rospy.sleep(tgap)
        pub_close.publish(Time(rospy.Time.now()))

def OpenTandem(robot):
	pub_open = rospy.Publisher('gripper_open', Time, queue_size=1)
        robot.arm.hand.MoveHand(0.1,0.1)
        pub_open.publish(Time(rospy.Time.now()))

def callback_grasp(data):
	global grasp_bool
	grasp_bool = data.data
	
rospy.init_node('test_scenario', anonymous = True)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.misc.InitOpenRAVELogging();
rospy.init_node('test_scenario', anonymous = True)
env, robot = adapy.initialize(attach_viewer='qtcoin', sim=False)
manip = robot.arm
manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
robot.arm.SetActive()

rospy.Subscriber('object_grasp_flag',Bool,callback_grasp)


from IPython import embed
embed()
