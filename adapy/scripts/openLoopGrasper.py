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

# 6 DOF on arm and 2 fingers = total 8
# from 0 to 7, 6 and 7 are fingers while 5 is the wrist
# self.robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
FINGER_ONE_INDEX = 6
FINGER_TWO_INDEX = 7
WRIST_INDEX = 5

class openLoopGrasper(object):

    def __init__(self,robot,environment):
        self.robot = robot
        self.env = environment
        self.trueInitTime = 0
        self.alphaLimits = [0.5,5.0]
        self.backFactorLimits = [1.,10.]
        self.NoILimits = [1.,10.]
        # this is used for grasp verification.
        # The message is published from another node in graspObjectChecker.py
        rospy.Subscriber('object_grasp_flag',Bool,self.callback_grasp)

    def CloseTandem(self):
        # self.pub_close = rospy.Publisher('gripper_close', Time, queue_size=1)
        self.robot.arm.hand.MoveHand(1.34,0.1)
        rospy.sleep(1)
        self.robot.arm.hand.MoveHand(1.34,1.34)
        rospy.sleep(0.1)
        # self.pub_close.publish(Time(rospy.Time.now()))

    def OpenTandem(self):
        # self.pub_open = rospy.Publisher('gripper_open', Time, queue_size=1)
        self.robot.arm.hand.MoveHand(0.1,0.1)
        # self.pub_open.publish(Time(rospy.Time.now()))

    def callback_grasp(self,data):
        if data.data == True:
            self.trueInitTime = rospy.get_time()
            self.grasp_bool = data.data #True
        elif rospy.get_time()-self.trueInitTime > 5:
            self.grasp_bool = data.data #False

        #self.grasp_bool = data.data

    def PolicyExecute(self,alpha,deltatime,backFactor,NoI):
        # if we move hand to 0.2, then we must use the sleep function after movehand
        # XXX: there is a bug in prpy, sometimes the robot will get to the next execution
        # before the robot complete an action
        # self.robot.arm.hand.MoveHand(0.2,0.2)
        # rospy.sleep(1)

        # 1. rotate wrist while closing fingers
        # after completely closed, open fingers fast and repeat
        self.traj = self.FreqPolicyToTraj(alpha,deltatime,backFactor,NoI)
        if self.traj.GetNumWaypoints() > 100:
            return (False,self.traj.GetNumWaypoints())
        self.Executeresult = self.robot.ExecuteTrajectory(self.traj, execute=True)
        rospy.sleep(1)
        # 2. close finger 1 but not finger 2
        # at the same time, graspChecker is running to gather data, analyze data, and verify grasp
        self.CloseTandem()
        rospy.sleep(5)
        self.OpenTandem()
        # in the end, after verifacation, return
        # TODO: this is temporarily without grasp verification
        self.grasp_bool = True
        return (self.grasp_bool,self.traj.GetNumWaypoints())
        #self.robot.arm.hand.MoveHand(1.2,1.2)

    # Three parameter policy
    def FreqPolicyToTraj(self,alpha,deltatime,backFactor,NoI):
        # wrist rotation speed
        d_rot = 0.1
        # alpha is the ratio between wrist rotation speed and the speed of finger closure and opening
        minConfig = 1.1
        self.Traj2DOF = []

        # 6 DOF on arm and 2 fingers = total 8
        # from 0 to 7, 6 and 7 are fingers while 5 is the wrist
        self.robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
        # Describes how the data should be interpolated.
        # how to interpolate between 2 points in a trajectory
        # http://openrave.org/docs/latest_stable/coreapihtml/classOpenRAVE_1_1ConfigurationSpecification_1_1Group.html
        c = self.robot.GetActiveConfigurationSpecification(interpolation='linear')
        # http://openrave.org/docs/0.8.2/architecture/trajectory/
        # adds the deltatime tag to the end (the 9th DOF)
        # if one doesn't exist and returns the index into the configuration space
        c.AddDeltaTimeGroup()

        traj = openravepy.RaveCreateTrajectory(env, '')
        traj.Init(c)

        # initconfig is an array with length = 8
        initConfig = self.robot.GetActiveDOFValues()
        # Stack arrays in sequence horizontally (column wise).
        # make initconfig from 8*1 to 9*1
        initConfig = np.hstack((initConfig,[deltatime]))

        # maxConfig is the finger config when it is open to the most
        maxConfig = initConfig[FINGER_ONE_INDEX]
        dFinger = minConfig-maxConfig

        currConfig = copy.deepcopy(initConfig)
        currInd = 0
        traj.Insert(currInd,currConfig)
        # this traj2DOF only consider the wrist
        # because in this case we don't need to move wrist and finger at the same time
        self.Traj2DOF.append([currConfig[FINGER_ONE_INDEX],currConfig[WRIST_INDEX]])
        for closeLooper in range(0,NoI):
            # d_rot control the wrist rotation speed and alpha*d_rot control the finger speed
            d_policy = np.array([0.,0.,0.,0.,0.,d_rot,-alpha*d_rot,-alpha*d_rot,0.])
            # while currConfig[FINGER_ONE_INDEX] < minConfig, then the finger is not closed completely yet
            # so we will keep closing it
            while currConfig[FINGER_ONE_INDEX]<minConfig:
                # rotate the wrist and close the fingers (finger config is increasing) at the same time
                currConfig = currConfig - d_policy
                # print currConfig
                currInd = currInd + 1
                traj.Insert(currInd,currConfig)
                self.Traj2DOF.append([currConfig[FINGER_ONE_INDEX],currConfig[WRIST_INDEX]])

            # make sure after all the iterations, the finger configs will be minConfig
            currConfig[FINGER_ONE_INDEX]=minConfig
            currConfig[FINGER_TWO_INDEX]=minConfig
            currInd = currInd + 1
            traj.Insert(currInd,currConfig)
            self.Traj2DOF.append([currConfig[FINGER_ONE_INDEX],currConfig[WRIST_INDEX]])

            # opening fingers should be much faster than closing fingers, therefore we have a backFactor
            d_policy = np.array([0.,0.,0.,0.,0.,d_rot,backFactor*alpha*d_rot,backFactor*alpha*d_rot,0.])
            while currConfig[FINGER_ONE_INDEX]>maxConfig:
                # rotate the wrist and open the fingers (finger config is decreasing) at the same time
                currConfig = currConfig - d_policy
                # print currConfig
                currInd = currInd + 1
                traj.Insert(currInd,currConfig)
                self.Traj2DOF.append([currConfig[FINGER_ONE_INDEX],currConfig[WRIST_INDEX]])
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
        # success is the bool result of grasp verification and trajLength is the number of waypoints
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
# openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Error)
openravepy.misc.InitOpenRAVELogging();
rospy.init_node('test_scenario', anonymous = True)
# env, robot = adapy.initialize(attach_viewer='qtcoin', sim=True)
env, robot = adapy.initialize(attach_viewer='interactivemarker', sim=True)
manip = robot.arm
manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
robot.SetActiveDOFs([0,1,2,3,4,5,6,7])
robot.arm.SetActive()


# Create class object for the openloopgrasper Example of testing a set of parameters
O = openLoopGrasper(robot,env)
# (success,trajLength) = O.PolicyExecute(1.5,1,10,2)
# print success,trajLength
# embed()
# exit(0)

# Now run UCB - GP
# Now we have 3 parameters, and we want to test all the possible values of those parameters.
# In order to simplify this process, we will normalize each parameter to [0,1]
# Each parameter after normalization will become a value between 0 to 1
# and our resolution for each parameter is 11.
# so param 1 after normalization = 0, 0.1, 0.2, ... 0.9, 1.0
# param 2 after normalization = 0, 0.1, 0.2, ... 0.9, 1.0
# param 3 after normalization = 0, 0.1, 0.2, ... 0.9, 1.0
# There are 11*11*11 combinations
# each combination of parameters will correspond to a grasping trajectory, then we can choose the best one

# num of parameters
feature_size = 3
# num of testing values for each parameter
resolution = 11
# we will test all the combinations of all the 3 parameters - there are 11*11*11
num_of_combinations = resolution**feature_size
# XTest will store all the combinations of parameters, its size = (11^3) * 3
XTest = np.zeros((num_of_combinations,feature_size))
# after normalization, the test values for each parameter = x_res = [0 0.1 0.2 ... 0.9 1.0]
x_res = np.linspace(0,1,resolution)
for x in range(0,num_of_combinations):
    for f in range(0,feature_size):
        XTest[x,f] = x_res[np.floor(x/(resolution**(feature_size-f-1)))%resolution]


epsilon = 0.2
# n
total_iterations = 20
# epsilon*n
no_iterations_simple = int(epsilon * total_iterations)
# (1-epsilon)*n
no_iterations_advanced = int((1-epsilon)*total_iterations)


graspData = []
# start with random parameters
seedX = np.array([random.uniform(0,1) for t in range(0,3)])
# execute a grasp with the 3 init parameters, and retrieve the information about the grasp back
(alpha,backFactor,NoI,success,trajLength) = O.executeParameters(seedX)
# now graspData = [[4.7498273609196149, 7.8699604314680691, 4, True, 36]]
graspData.append([alpha,backFactor,NoI,success,trajLength])
print 'Evaluating Seed'
# View inputs as arrays with at least two dimensions.
# it will transform seedX=array([ 0.94440608,  0.76332894,  0.44268201]) into
# X=array([[ 0.94440608,  0.76332894,  0.44268201 ]])
X = np.atleast_2d(seedX)
# return the fitness score value of this grasp
# if the number of waypoints is very low, then it will return a big score
# otherwise, it will return a low score
y = np.atleast_2d(O.fitness(success,trajLength))
# X = [[3 parameters]] and y = fittness score
print '0. alpha = {0}, backFactor = {1}, NoI = {2}, success = {3}, trajLength = {4}, Fitness = {5} '\
        .format(alpha,backFactor,NoI,success,trajLength,O.fitness(success,trajLength))



print "Begining stage 1"
for n in range(0,no_iterations_simple):
    model = pyGPs.GPR()
    # X = [[3 parameters]] and y = fittness score
    model.setData(X,y)
    # ym = predictive mean = array of the means of all the ys
    # ys2 = predictive variance = array of the variance of all the ys
    # fm = latent mean
    # fs2 = latent variance
    # lp = log predictive probability
    ym, ys2, fmu, fs2, lp = model.predict(XTest)
    # std dev
    ys = np.sqrt(ys2)
    # Now we want the parameters with the highest variance or the highest uncertainty
    # We will test these cases
    UCB = ys
    # numpy.argmax() - Returns the indices of the maximum values along an axis.
    # idMaxUCB is the index of the parameter combination which generates the max variance
    idMaxUCB = UCB.argmax()

    # then we only care about the parameter combination with max variance and test it
    curX = XTest[idMaxUCB]
    raw_input('Press Enter to continue grasp')
    (alpha,backFactor,NoI,success,trajLength) = O.executeParameters(curX)
    graspData.append([alpha,backFactor,NoI,success,trajLength])

    X = np.append(X,np.atleast_2d(curX),0)
    y = np.append(y,np.atleast_2d(O.fitness(success,trajLength)),0)

    print '{6}. alpha = {0}, backFactor = {1}, NoI = {2}, success = {3}, trajLength = {4}, Fitness = {5} '.format(alpha,backFactor,NoI,success,trajLength,O.fitness(success,trajLength),n)

# Evaluations with hyperparameter optimization
for n in range(no_iterations_simple,no_iterations_simple + no_iterations_advanced):
    # start from a new model
    model = pyGPs.GPR()

    model.optimize(X, y)
    ym, ys2, fmu, fs2, lp = model.predict(XTest)
    ys = np.sqrt(ys2)
    # the difference between this one and the previous loop
    # Now we want to choose the parameter with the highest mean+3*stddev
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
embed()

