#!/usr/bin/env python
import adapy, openravepy, numpy
from IPython import embed


openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.misc.InitOpenRAVELogging();

env, robot = adapy.initialize(attach_viewer='rviz', sim=False)
manip = robot.arm
manip.SetIkSolver(openravepy.RaveCreateIkSolver(env, 'NloptIK'))
#joint_values1 = manip.GetDOFValues()
#joint_values2 = manip.GetDOFValues()
#embed()
#robot.SetActiveDOFs([0,1,2,3,4,5])
#robot.SetActiveDOFs([0])
joint_values1 = robot.GetActiveDOFValues()
#joint_values2 = robot.GetActiveDOFValues()
#joint_values1[0] = joint_values1[0] - 0.5;
#joint_values1[3] = joint_values1[3] + 0.3;
#joint_values2[2] = joint_values2[2];
#joint_values2[3] = joint_values2[3];
embed()

#import time

#dof_indices=self.GetArmIndices(), affine_dofs=0, simulated=sim)    
#from IPython import embed
#embed()


#test 1 planner
#curr_values = robot.GetActiveDOFValues();
#traj = openravepy.RaveCreateTrajectory(env, '')
#traj.Init(robot.GetActiveConfigurationSpecification())
#traj.Insert(0,curr_values)
#traj.Insert(1,joint_values1)
#traj.Insert(2,joint_values2)
#openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
#robot.arm.controller.SetPath(traj)
# embed()
#robot.controller.SetPath(joint_values2)
#robot.PlanToConfiguration(joint_values1)
#time.sleep(2)
#robot.PlanToConfiguration(joint_values2)
#robot.controller.SetPath(joint_values1)

#embed()
# print "testing ik_planner"
# controller = robot.GetController()
# traj1 = robot.ik_planner.PlanToConfiguration(robot,joint_values1)
# robot.SetPath(traj1)
# time.sleep(1)
# traj2 = robot.ik_planner.PlanToConfiguration(robot,joint_values2)
# robot.SetPath(traj2)

#print "testing named_planner"
#controller = robot.GetController()
#traj1 = robot.named_planner.PlanToConfiguration(robot,joint_values1)
#controller.SetPath(traj1)
#time.sleep(1)
#embed()
#traj2 = robot.named_planner.PlanToConfiguration(robot,joint_values2)
#robot.SetPath(traj2)

#print "testing snap_planner"
#controller = robot.GetController()
#traj1 = robot.snap_planner.PlanToConfiguration(robot,joint_values1)
#controller.SetPath(traj1)
#time.sleep(1)
#traj2 = robot.snap_planner.PlanToConfiguration(robot,joint_values2)
#controller.SetPath(traj2)
#error: distance to goal larger than snap tolerance

#print "testing ompl_planner"
#controller = robot.GetController()
#traj1 = robot.omplplanner.PlanToConfiguration(robot,joint_values1)
#controller.SetPath(traj1)
#time.sleep(1)
#traj2 = robot.ompl_planner.PlanToConfiguration(robot,joint_values2)
#controller.SetPath(traj2)
#crashes

#print "testing chomp_planner"
#controller = robot.GetController()
#traj1 = robot.chomp_planner.PlanToConfiguration(robot,joint_values1)
#controller.SetPath(traj1)
#time.sleep(1)
#traj2 = robot.chomp_planner.PlanToConfiguration(robot,joint_values2)
#controller.SetPath(traj2)

# print "testing mk_planner"
# controller = robot.GetController()
# traj1 = robot.mk_planner.PlanToConfiguration(robot,joint_values1)
# controller.SetPath(traj1)
# time.sleep(1)
# traj2 = robot.mk_planner.PlanToConfiguration(robot,joint_values2)
# controller.SetPath(traj2)

#print "testing cbirrt_planner"
#embed()
#controller = robot.GetController()
t#raj1 = robot.cbirrt_planner.PlanToConfiguration(robot,joint_values1)
#controller.SetPath(traj1)
#embed();

#robot.SetPath
#.PlanToConfiguration(joint_values)
#time.sleep(3)
#controller.SetPath(traj)
#robot.PlanToConfiguration(joint_values)
#embed()
