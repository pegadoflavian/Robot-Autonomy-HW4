import logging, openravepy

import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy

class GraspPlanner(object):

		def __init__(self, robot, base_planner, arm_planner):
				self.robot = robot
				self.base_planner = base_planner
				self.arm_planner = arm_planner
				self.env = self.robot.GetEnv()
				self.manip = self.robot.GetActiveManipulator()

						
		def GetBasePoseForObjectGrasp(self, obj):

			# Load grasp database
			self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
			if not self.gmodel.load():
					self.gmodel.autogenerate()
			print "this is working"
			self.graspindices = self.gmodel.graspindices
			self.grasps = self.gmodel.grasps
			self.order_grasps()
			self.show_grasp(self.grasps_ordered[-1])
			base_pose = None
			grasp_config = None
			###################################################################
			# TODO: Here you will fill in the function to compute
			#  a base pose and associated grasp config for the 
			#  grasping the bottle
			###################################################################
			

			return base_pose, grasp_config

		# def PlanToGrasp(self, obj):

		# 		# Next select a pose for the base and an associated ik for the arm
		# 		base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

		# 		if base_pose is None or grasp_config is None:
		# 				print 'Failed to find solution'
		# 				exit()

		# 		# Now plan to the base pose
		# 		start_pose = np.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
		# 		base_plan = self.base_planner.Plan(start_pose, base_pose)
		# 		base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

		# 		print 'Executing base trajectory'
		# 		self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

		# 		# Now plan the arm to the grasp configuration
		# 		start_config = np.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
		# 		arm_plan = self.arm_planner.Plan(start_config, grasp_config)
		# 		arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

		# 		print 'Executing arm trajectory'
		# 		self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

		# 		# Grasp the bottle
		# 		task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
		# 		task_manipultion.CloseFingers()
		# def problem_init(self):
			
		# 	self.target_kinbody = self.env.ReadKinBodyURI('models/objects/fuze_bottle.iv')
		# 	print'choosing model'
		# 	#change the location so it's not under the robot
		# 	T = self.target_kinbody.GetTransform()
		# 	T[0:3,3] += np.array([0.5, 0.5, 0.5])
		# 	self.target_kinbody.SetTransform(T)
		# 	self.env.AddKinBody(self.target_kinbody)

		def eval_grasp(self, grasp):
			with self.robot:
				#contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
				try:
					contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

					obj_position = self.gmodel.target.GetTransform()[0:3,3]
					# for each contact

					G = np.array([]) #the wrench matrix
					i = 0
					for c in contacts:

						pos = c[0:3] - obj_position
						dir = -c[3:] #this is already a unit vector
						tempCross = np.cross(pos,dir)
						curWrench = np.concatenate((dir,tempCross),axis=0)
						curWrench = np.reshape(curWrench,(6,1))

						if i == 0:
							G = curWrench
							i = 1
						else:
							G = np.concatenate((G,curWrench),axis = 1)

					#print(np.shape(G))
					u,s,v = np.linalg.svd(G)

					#print np.shape(s)
					sMin = np.min(s)
					#TODO use G to compute scrores as discussed in class
					return -sMin #change this

				except openravepy.planning_error,e:
					#you get here if there is a failure in planning
					#example: if the hand is already intersecting the object at the initial position/orientation
					return  0.00 # TODO you may want to change this

		def order_grasps(self):
			self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
			
			for grasp in self.grasps_ordered:
				grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
			
			# sort!
			order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
			order = order[::-1]
			self.grasps_ordered = self.grasps_ordered[order]

		def show_grasp(self, grasp, delay=10):
			with openravepy.RobotStateSaver(self.gmodel.robot):
				with self.gmodel.GripperVisibility(self.gmodel.manip):
					time.sleep(0.1) # let viewer update?
					try:
						with self.env:
							contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
							#if mindist == 0:
							#  print 'grasp is not in force closure!'
							contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
							self.gmodel.robot.GetController().Reset(0)
							self.gmodel.robot.SetDOFValues(finalconfig[0])
							self.gmodel.robot.SetTransform(finalconfig[1])
							self.env.UpdatePublishedBodies()
							time.sleep(delay)
					except openravepy.planning_error,e:
						print 'bad grasp!',e
# OpenRAVE
'''
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata


#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
	if openrave_data_path == '':
			os.environ['OPENRAVE_DATA'] = ordata_path_thispack
	else:
			datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
			os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()



class RoboHandler:
	def __init__(self):
		#self.openrave_init()
		self.problem_init()

		#order grasps based on your own scoring metric
		self.order_grasps()

		#order grasps with noise
		#self.order_grasps_noisy()

		print 'showing best grasp'
		self.show_grasp(self.grasps_ordered[-1])
		raw_input('press any key to continue')

		#print 'showing second best grasp'
		#self.show_grasp(self.grasps_ordered[-2],30)
		#raw_input('press any key to continue')

		#print 'showing third best grasp'
		#self.show_grasp(self.grasps_ordered[-3])
		#print 'showing fourth best grasp'

		#raw_input('press any key to continue')
		#self.show_grasp(self.grasps_ordered[-4])
		#raw_input('press any key to continue')
	# the usual initialization for openrave
	def openrave_init(self):
		self.env = openravepy.Environment()
		self.env.SetViewer('qtcoin')
		self.env.GetViewer().SetName('HW1 Viewer')
		self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
		# time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
		self.robot = self.env.GetRobots()[0]
		self.manip = self.robot.GetActiveManipulator()
		self.end_effector = self.manip.GetEndEffector()

	# problem specific initialization - load target and grasp module
	def problem_init(self):
		self.target_kinbody = self.env.ReadKinBodyURI('models/objects/fuze_bottle.iv')
		#self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
		#self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')
		print'choosing model'
		#change the location so it's not under the robot
		T = self.target_kinbody.GetTransform()
		T[0:3,3] += np.array([0.5, 0.5, 0.5])
		self.target_kinbody.SetTransform(T)
		self.env.AddKinBody(self.target_kinbody)

		# create a grasping module
		self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)
		
		# if you want to set options, e.g. friction
		options = openravepy.options
		options.friction = 0.1
		if not self.gmodel.load():
			self.gmodel.autogenerate(options)

		self.graspindices = self.gmodel.graspindices
		self.grasps = self.gmodel.grasps

	
	# order the grasps - call eval grasp on each, set the 'performance' index, and sort
	def order_grasps(self):
		self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
		
		for grasp in self.grasps_ordered:
			grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
		
		# sort!
		order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
		order = order[::-1]
		self.grasps_ordered = self.grasps_ordered[order]
		
	
	# order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
	def order_grasps_noisy(self):
		self.grasps_ordered_noisy = self.grasps_ordered.copy() #you should change the order of self.grasps_ordered_noisy
		#TODO set the score with your evaluation function (over random samples) and sort


	# function to evaluate grasps
	# returns a score, which is some metric of the grasp
	# higher score should be a better grasp
	def eval_grasp(self, grasp):
		with self.robot:
			#contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
			try:
				contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

				obj_position = self.gmodel.target.GetTransform()[0:3,3]
				# for each contact

				G = np.array([]) #the wrench matrix
				i = 0
				for c in contacts:

					pos = c[0:3] - obj_position
					dir = -c[3:] #this is already a unit vector
					tempCross = np.cross(pos,dir)
					
					curWrench = np.concatenate((dir,tempCross),axis=0)
					curWrench = np.reshape(curWrench,(6,1))

					if i == 0:
						G = curWrench
						i = 1
					else:
						G = np.concatenate((G,curWrench),axis = 1)

				#print(np.shape(G))
				u,s,v = np.linalg.svd(G)

				#print np.shape(s)
				sMin = np.min(s)
				#TODO use G to compute scrores as discussed in class
				return -sMin #change this

			except openravepy.planning_error,e:
				#you get here if there is a failure in planning
				#example: if the hand is already intersecting the object at the initial position/orientation
				return  0.00 # TODO you may want to change this
class RoboHandler:
	def __init__(self):
		self.openrave_init()
		self.problem_init()

		#order grasps based on your own scoring metric
		self.order_grasps()

		print 'showing best grasp'
		self.show_grasp(self.grasps_ordered[-1])
		raw_input('press any key to continue')

		print 'showing second best grasp'
		self.show_grasp(self.grasps_ordered[-2],30)
		raw_input('press any key to continue')

		print 'showing third best grasp'
		self.show_grasp(self.grasps_ordered[-3])
		print 'showing fourth best grasp'

		raw_input('press any key to continue')
		self.show_grasp(self.grasps_ordered[-4])
		raw_input('press any key to continue')
	# the usual initialization for openrave
	def openrave_init(self):
		self.env = openravepy.Environment()
		self.env.SetViewer('qtcoin')
		self.env.GetViewer().SetName('HW1 Viewer')
		self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
		# time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
		self.robot = self.env.GetRobots()[0]
		self.manip = self.robot.GetActiveManipulator()
		self.end_effector = self.manip.GetEndEffector()

	# problem specific initialization - load target and grasp module
	def problem_init(self):
		self.target_kinbody = self.env.ReadKinBodyURI('models/objects/fuze_bottle.iv')
		#self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
		#self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')
		print'choosing model'
		#change the location so it's not under the robot
		T = self.target_kinbody.GetTransform()
		T[0:3,3] += np.array([0.5, 0.5, 0.5])
		self.target_kinbody.SetTransform(T)
		self.env.AddKinBody(self.target_kinbody)

		# create a grasping module
		self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)
		
		# if you want to set options, e.g. friction
		options = openravepy.options
		options.friction = 0.1
		if not self.gmodel.load():
			self.gmodel.autogenerate(options)

		self.graspindices = self.gmodel.graspindices
		self.grasps = self.gmodel.grasps

	
	# order the grasps - call eval grasp on each, set the 'performance' index, and sort
	def order_grasps(self):
		self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
		
		for grasp in self.grasps_ordered:
			grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
		
		# sort!
		order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
		order = order[::-1]
		self.grasps_ordered = self.grasps_ordered[order]
		
	
	# order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
	def order_grasps_noisy(self):
		self.grasps_ordered_noisy = self.grasps_ordered.copy() #you should change the order of self.grasps_ordered_noisy
		#TODO set the score with your evaluation function (over random samples) and sort


	# function to evaluate grasps
	# returns a score, which is some metric of the grasp
	# higher score should be a better grasp
	def eval_grasp(self, grasp):
		with self.robot:
			#contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
			try:
				contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

				obj_position = self.gmodel.target.GetTransform()[0:3,3]
				# for each contact

				G = np.array([]) #the wrench matrix
				i = 0
				for c in contacts:

					pos = c[0:3] - obj_position
					dir = -c[3:] #this is already a unit vector
					tempCross = np.cross(pos,dir)
					
					curWrench = np.concatenate((dir,tempCross),axis=0)
					curWrench = np.reshape(curWrench,(6,1))

					if i == 0:
						G = curWrench
						i = 1
					else:
						G = np.concatenate((G,curWrench),axis = 1)

				#print(np.shape(G))
				u,s,v = np.linalg.svd(G)

				#print np.shape(s)
				sMin = np.min(s)
				#TODO use G to compute scrores as discussed in class
				return -sMin #change this

			except openravepy.planning_error,e:
				#you get here if there is a failure in planning
				#example: if the hand is already intersecting the object at the initial position/orientation
				return  0.00 # TODO you may want to change this
			
			#heres an interface in case you want to manipulate things more specifically
			#NOTE for this assignment, your solutions cannot make use of graspingnoise
#      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
#      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
#      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
#      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
#      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
#            direction             = grasp[self.graspindices.get('igraspdir')], \
#            roll                  = grasp[self.graspindices.get('igrasproll')], \
#            position              = grasp[self.graspindices.get('igrasppos')], \
#            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
#            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
#            target                = self.target_kinbody, \
#            graspingnoise         = 0.0, \
#            forceclosure          = True, \
#            execute               = False, \
#            outputfinal           = True, \
#            translationstepmult   = None, \
#            finestep              = None )



	# given grasp_in, create a new grasp which is altered randomly
	# you can see the current position and direction of the grasp by:
	# grasp[self.graspindices.get('igrasppos')]
	# grasp[self.graspindices.get('igraspdir')]
	def sample_random_grasp(self, grasp_in):
		grasp = grasp_in.copy()

		#sample random position
		RAND_DIST_SIGMA = 0.01 #TODO you may want to change this
		pos_orig = grasp[self.graspindices['igrasppos']]
		#TODO set a random position


		#sample random orientation
		RAND_ANGLE_SIGMA = np.pi/24 #TODO you may want to change this
		dir_orig = grasp[self.graspindices['igraspdir']]
		roll_orig = grasp[self.graspindices['igrasproll']]
		#TODO set the direction and roll to be random

		return grasp


	#displays the grasp
	def show_grasp(self, grasp, delay=2):
		with openravepy.RobotStateSaver(self.gmodel.robot):
			with self.gmodel.GripperVisibility(self.gmodel.manip):
				time.sleep(0.1) # let viewer update?
				try:
					with self.env:
						contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
						#if mindist == 0:
						#  print 'grasp is not in force closure!'
						contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
						self.gmodel.robot.GetController().Reset(0)
						self.gmodel.robot.SetDOFValues(finalconfig[0])
						self.gmodel.robot.SetTransform(finalconfig[1])
						self.env.UpdatePublishedBodies()
						time.sleep(delay)
				except openravepy.planning_error,e:
					print 'bad grasp!',e
'''