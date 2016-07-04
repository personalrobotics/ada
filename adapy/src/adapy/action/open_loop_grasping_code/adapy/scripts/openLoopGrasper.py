#!/usr/bin/env python
import adapy, openravepy, numpy, prpy, rospy
from IPython import embed
import numpy as np
import copy
from std_msgs.msg import Time
from std_msgs.msg import Bool
import random
import pyGPs
import matplotlib.pyplot as plt
class openLoopGrasper(object):

	def __init__(self,robot,environment):
		self.robot = robot
		self.env = environment
		self.trueInitTime = 0
		self.alphaLimits = [0.5,5.0]
		self.backFactorLimits = [1.,10.]
		self.NoILimits = [1.,10.]
		rospy.Subscriber('object_grasp_flag',Bool,self.callback_grasp)

	def CloseTandem(self):
        	self.pub_close = rospy.Publisher('gripper_close', Time, queue_size=1)
        	self.robot.arm.hand.MoveHand(1.34,0.1)
		rospy.sleep(1)
        	self.robot.arm.hand.MoveHand(1.34,1.34)
        	rospy.sleep(0.1)
        	self.pub_close.publish(Time(rospy.Time.now()))

	def OpenTandem(self):
		self.pub_open = rospy.Publisher('gripper_open', Time, queue_size=1)
	        self.robot.arm.hand.MoveHand(0.1,0.1)
	        self.pub_open.publish(Time(rospy.Time.now()))

	def callback_grasp(self,data):
		if data.data == True:
			self.trueInitTime = rospy.get_time()
			self.grasp_bool = data.data #True
		elif rospy.get_time()-self.trueInitTime > 5:
			self.grasp_bool = data.data #False

		#self.grasp_bool = data.data

	def PolicyExecute(self,alpha,deltatime,backFactor,NoI):
		self.robot.arm.hand.MoveHand(0.2,0.2)
		self.traj = self.FreqPolicyToTraj(alpha,deltatime,backFactor,NoI)
		if self.traj.GetNumWaypoints() > 100:
			return (False,self.traj.GetNumWaypoints())
		self.Executeresult = self.robot.ExecuteTrajectory(self.traj)
		rospy.sleep(1)
		self.CloseTandem()
		rospy.sleep(5)
		self.OpenTandem()
		return (self.grasp_bool,self.traj.GetNumWaypoints())
		#self.robot.arm.hand.MoveHand(1.2,1.2)
	
	def FreqPolicyToTraj(self,alpha,deltatime,backFactor,NoI):
		d_rot = 0.1
		minConfig = 1.1
		self.Traj2DOF = []
		self.robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
		c = self.robot.GetActiveConfigurationSpecification(interpolation='linear')
		c.AddDeltaTimeGroup()
		traj = openravepy.RaveCreateTrajectory(env, '')
		traj.Init(c)
		initConfig = self.robot.GetActiveDOFValues()
		initConfig = np.hstack((initConfig,[deltatime]))
		maxConfig = initConfig[0]
		dFinger = minConfig-maxConfig
		d_policy = np.array([-alpha*d_rot,-alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
		currConfig = copy.deepcopy(initConfig)
		currInd = 0
		traj.Insert(currInd,currConfig)
		self.Traj2DOF.append([currConfig[0],currConfig[7]])
		for closeLooper in range(0,NoI):
			d_policy = np.array([-alpha*d_rot,-alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
			while currConfig[0]<minConfig:
				currConfig = currConfig - d_policy
				#print currConfig
				currInd = currInd + 1
				traj.Insert(currInd,currConfig)
				self.Traj2DOF.append([currConfig[0],currConfig[7]])
			currConfig[0]=minConfig
			currConfig[1]=minConfig
			currInd = currInd + 1
			traj.Insert(currInd,currConfig)
			self.Traj2DOF.append([currConfig[0],currConfig[7]])
			
			d_policy = np.array([backFactor*alpha*d_rot,backFactor*alpha*d_rot,0.,0.,0.,0.,0.,d_rot,0.])
			while currConfig[0]>maxConfig:
				currConfig = currConfig - d_policy
				#print currConfig
				currInd = currInd + 1
				traj.Insert(currInd,currConfig)
				self.Traj2DOF.append([currConfig[0],currConfig[7]])
		self.Traj2DOF = np.array(self.Traj2DOF)
		return traj

	def normalizeParameters(self,alphaUN,backFactorUN,NoIUN):
		alpha = (alphaUN - self.alphaLimits[0])/(self.alphaLimits[1]-self.alphaLimits[0])
		backFactor = (backFactorUN - self.backFactorLimits[0])/(self.backFactorLimits[1]-self.backFactorLimits[0])
		NoI = (NoIUN - self.NoILimits[0])/(self.NoILimits[1] - self.NoILimits[0])
		return (alpha,backFactor,NoI)
	
	def unNormalizeParameters(self,alphaN,backFactorN,NoIN):
		alpha = alphaN*(self.alphaLimits[1]-self.alphaLimits[0]) + self.alphaLimits[0]
		backFactor = backFactorN*(self.backFactorLimits[1]-self.backFactorLimits[0]) + self.backFactorLimits[0]
		NoI = int(NoIN*(self.NoILimits[1]-self.NoILimits[0]) + self.NoILimits[0])
		return (alpha,backFactor,NoI)

	def splitParameters(self,x):
		alpha = x[0]
		backFactor = x[1]
		NoI = x[2]
		return (alpha,backFactor,NoI)

	def executeParameters(self,x):
		(alphaN,backFactorN,NoIN) = self.splitParameters(x)
		(alpha,backFactor,NoI) = self.unNormalizeParameters(alphaN,backFactorN,NoIN)
		deltatime = 1
		(success,trajLength) = self.PolicyExecute(alpha,deltatime,backFactor,NoI)
		print 'Executing ',alpha,backFactor,NoI
		return (alpha,backFactor,NoI,success,trajLength)

	def fitness(self,success,trajLength):
		if success == False:
			return -10.0
		else:
			return 2.0 - (trajLength/100.0)
# Initialise stuff
rospy.init_node('test_scenario', anonymous = True)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.misc.InitOpenRAVELogging();
rospy.init_node('test_scenario', anonymous = True)
env, robot = adapy.initialize(attach_viewer='qtcoin', sim=False)
manip = robot.arm
manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
robot.arm.SetActive()


# Create class object for the openloopgrasper Example of testing a set of parameters
O = openLoopGrasper(robot,env)
#(success,trajLength) = O.PolicyExecute(1.5,1,10,2)
#print success,trajLength
#embed()
# Now run UCB - GP
feature_size = 3
resolution = 11
no_elements = resolution**feature_size
XTest = np.zeros((no_elements,feature_size))
x_res = np.linspace(0,1,resolution)
epsilon = 0.2
total_iterations = 20
no_iterations_simple = int(epsilon * total_iterations)
no_iterations_advanced = int((1-epsilon)*total_iterations)

for x in range(0,no_elements):
	for f in range(0,feature_size):
		XTest[x,f] = x_res[np.floor(x/(resolution**(feature_size-f-1)))%resolution]

graspData = []
seedX = np.array([random.uniform(0,1) for t in range(0,3)])
(alpha,backFactor,NoI,success,trajLength) = O.executeParameters(seedX)

graspData.append([alpha,backFactor,NoI,success,trajLength])
print 'Evaluating Seed'
X = np.atleast_2d(seedX)
y = np.atleast_2d(O.fitness(success,trajLength))

print '0. alpha = {0}, backFactor = {1}, NoI = {2}, success = {3}, trajLength = {4}, Fitness = {5} '.format(alpha,backFactor,NoI,success,trajLength,O.fitness(success,trajLength))

print "Begining stage 1"
for n in range(0,no_iterations_simple):
	  model = pyGPs.GPR()
	  model.setData(X,y)
	  ym, ys2, fmu, fs2, lp = model.predict(XTest)
	  ys = np.sqrt(ys2)#std
	  UCB = ys
	  idMaxUCB = UCB.argmax()

	  curX = XTest[idMaxUCB]
	  raw_input('Press Enter to continue grasp')
	  (alpha,backFactor,NoI,success,trajLength) = O.executeParameters(curX)
	  graspData.append([alpha,backFactor,NoI,success,trajLength])

	  X = np.append(X,np.atleast_2d(curX),0)
	  y = np.append(y,np.atleast_2d(O.fitness(success,trajLength)),0)

	  print '{6}. alpha = {0}, backFactor = {1}, NoI = {2}, success = {3}, trajLength = {4}, Fitness = {5} '.format(alpha,backFactor,NoI,success,trajLength,O.fitness(success,trajLength),n)

## Evaluations with hyperparameter optimization
for n in range(no_iterations_simple,no_iterations_simple + no_iterations_advanced):
	  model = pyGPs.GPR()           # start from a new model

	  model.optimize(X, y)
	  ym, ys2, fmu, fs2, lp = model.predict(XTest)
	  ys = np.sqrt(ys2)#std
	  UCB = ym + 3*ys
	  idMaxUCB = UCB.argmax()

	  curX = XTest[idMaxUCB]
	  raw_input('Press Enter to continue grasp')
          (alpha,backFactor,NoI,success,trajLength) = O.executeParameters(curX)
          graspData.append([alpha,backFactor,NoI,success,trajLength])

          X = np.append(X,np.atleast_2d(curX),0)
          y = np.append(y,np.atleast_2d(O.fitness(success,trajLength)),0)

          print '{6}. alpha = {0}, backFactor = {1}, NoI = {2}, success = {3}, trajLength = {4}, Fitness = {5} '.format(alpha,backFactor,NoI,success,trajLength,O.fitness(success,trajLength),n)


np.save('/home/lerrelp/data/{0}'.format(str(int(rospy.get_time()))),graspData)
from IPython import embed
embed()

