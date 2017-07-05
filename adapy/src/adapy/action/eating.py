#!/usr/bin/env python
import logging, numpy
from prpy.action import ActionMethod, ActionError
from prpy.planning.base import PlanningError
logger = logging.getLogger('adapy')
import prpy
import IPython
from tf.transformations import *

@ActionMethod
def BringToMouth(robot, mouth_position, fork=None, manip=None):
	"""
	@param robot the robot performing the motion
	@param mouth_position the world coordinates of the mouth (x,y,z)
	@param fork the fork kinbody - if None, will get from environment
	@param manip the manipulator - if None, will use the active one
	"""
	# Get the fork kinbody
	env = robot.GetEnv()
	if fork is None:
		fork = env.GetKinBody('forque')
		if fork is None:
			raise ActionError("Failed to find fork in the robot's environment")
			
	# Get the manipulator
	if manip is None:
		with robot.GetEnv():
			manip = robot.GetActiveManipulator()
			
	# fork pointed towards the person (TODO: make the normal direction an input)
	desired_fork_tip_in_world = rotation_matrix(numpy.pi/2,[1.0,0,0])
	desired_fork_tip_in_world[0,3] = mouth_position[0]
	desired_fork_tip_in_world[1,3] = mouth_position[1]
	desired_fork_tip_in_world[2,3] = mouth_position[2]
	
	# transform desired fork tip into desired end effector pose
	fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
	ee_in_world = manip.GetEndEffectorTransform()
	ee_in_fork_tip = numpy.dot(numpy.linalg.inv(fork_tip_in_world), ee_in_world)
	desired_ee_pose = numpy.dot(desired_fork_tip_in_world, ee_in_fork_tip)   
	
	# plan to the mouth pose
	try:
		with prpy.viz.RenderPoses([fork_tip_in_world, desired_fork_tip_in_world], env):
			path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=True)
	except PlanningError, e:
		raise ActionError('Failed to plan to mouth pose: %s' % str(e))

@ActionMethod
def MoveToRest(robot):
	"""
	@param robot the robot performing the motion
	"""
	robot.PlanToNamedConfiguration('eating_rest',execute=True)
	
@ActionMethod
def SkewerFromPlate(robot, goal_position, z_offset=None, plate=None, fork=None, manip=None):
	"""
	@param robot The robot performing the skewering
	@param plate the plate kinbody
	@param goal_position the (x,y) position of the bite w.r.t. to the center 
		of the place
	@param z_offset the height from the bottom of the plate (i.e. plate 
		thickness + desired safety margin) - if None the height of the
		provided plate kinbody is used
	@param fork fork kinbody - if None, will be read from robot's env
	@param manip robot manipulator - if None will use the active one
	"""
	
	# Get the fork kinbody
	env = robot.GetEnv()
	if fork is None:
		fork = env.GetKinBody('forque')
		if fork is None:
			raise ActionError("Failed to find fork in the robot's environment")

	# Get the plate kinbody
	if plate is None:
		plate = env.GetKinBody('plate')
		if plate is None:
			raise ActionError("Failed to find plate in the robot's environment")
				
	# Set default z_offset if not provided
	if z_offset is None:
		z_offset = 0.02
	
	# Get the manipulator
	if manip is None:
		with robot.GetEnv():
			manip = robot.GetActiveManipulator()

			
	# fork pointed downward
	desired_fork_tip_in_world = numpy.array([[-1.,  0., 0., 0.],
											 [ 0.,  1., 0., 0.],
											 [ 0.,  0.,-1., 0.],
											 [ 0.,  0., 0., 1.]])
	Rz = rotation_matrix(numpy.pi, [0,0,1])
	desired_fork_tip_in_world = numpy.dot(desired_fork_tip_in_world, Rz)
	
	# calculate desired fork position to be above the plate
	z_above_plate = 0.05
	y_fork_offset = -0.02 # for the bending of the tines
	plate_pose = plate.GetTransform()
	desired_fork_tip_in_world[0,3] = plate_pose[0,3] + goal_position[0]
	desired_fork_tip_in_world[1,3] = plate_pose[1,3] + goal_position[1] + y_fork_offset
	desired_fork_tip_in_world[2,3] = plate_pose[2,3] + z_offset + z_above_plate
	
	# transform desired fork tip into desired end effector pose
	fork_tip_in_world = fork.GetLink('tinetip').GetTransform()
	ee_in_world = manip.GetEndEffectorTransform()
	ee_in_fork_tip = numpy.dot(numpy.linalg.inv(fork_tip_in_world), ee_in_world)
	desired_ee_pose = numpy.dot(desired_fork_tip_in_world, ee_in_fork_tip)   

	# plan to above the morsal
	try:
		with prpy.viz.RenderPoses([fork_tip_in_world, desired_fork_tip_in_world], env):
			path = robot.PlanToEndEffectorPose(desired_ee_pose, execute=True)
	except PlanningError, e:
		raise ActionError('Failed to plan to pose above morsal: %s' % str(e))

	# move downward until reaching the offset provided
	try:
		direction = numpy.array([0., 0., -1.])
		distance = z_above_plate
		with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3], direction=direction, length=distance, env=robot.GetEnv()):
			with prpy.rave.Disabled(fork):
				T = robot.arm.GetEndEffectorTransform()
				path = robot.arm.PlanToEndEffectorOffset(direction=direction, distance=distance, execute=True)
	except PlanningError, e:
		raise ActionError('Failed to plan straight line path to grab morsal: %s' % str(e))
	
	# At the end of the motion, get the final fork tinetip location
	final_fork_transform = fork.GetLink('tinetip').GetTransformPose()
	
	# move back up to the offset provided
	try:
		direction = numpy.array([0., 0., 1.])
		distance = z_above_plate
		with prpy.viz.RenderVector(manip.GetEndEffectorTransform()[:3,3], direction=direction, length=distance, env=robot.GetEnv()):
			with prpy.rave.Disabled(fork):
				T = robot.arm.GetEndEffectorTransform()
				path = robot.arm.PlanToEndEffectorOffset(direction=direction, distance=distance, execute=True)
	except PlanningError, e:
		raise ActionError('Failed to plan straight line path to grab morsal: %s' % str(e))
		
	return final_fork_transform
