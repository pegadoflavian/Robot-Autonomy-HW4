import numpy
import math
import time
import random

class HerbEnvironment(object):
	
	def __init__(self, herb):
		self.robot = herb.robot

		# add a table and move the robot into place
		table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
		self.robot.GetEnv().Add(table)

		table_pose = numpy.array([[ 0, 0, -1, 0.6], 
								  [-1, 0,  0, 0], 
								  [ 0, 1,  0, 0], 
								  [ 0, 0,  0, 1]])
		table.SetTransform(table_pose)

		# set the camera
		camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
								   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
								   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
								   [ 0.        ,  0.        ,  0.        ,  1.        ]])
		self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
		
		# goal sampling probability
		self.p = 0.0

	def SetGoalParameters(self, goal_config, p = 0.2):
		self.goal_config = goal_config
		self.p = p
		

	def GenerateRandomConfiguration(self):
		config = [0] * len(self.robot.GetActiveDOFIndices())

		lower_limits, upper_limits = self.robot.GetActiveDOFLimits()

		import numpy
		lower_limits = numpy.array(lower_limits)
		upper_limits = numpy.array(upper_limits)

		# Generate random configuration
		choice = numpy.random.rand(1)
		if choice < self.p:
			config = self.goal_config
		else:
			COLLISION = True
			while COLLISION:
				config = numpy.random.rand(len(self.robot.GetActiveDOFIndices()))*(upper_limits - lower_limits) + lower_limits
				# Check if it is collision free
				with self.robot:
					robot_pos = self.robot.GetActiveDOFValues()
					robot_pos = config
					self.robot.SetActiveDOFValues(robot_pos)
					if (self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()) == False:
						COLLISION = False
		return numpy.array(config)


	
	def ComputeDistance(self, start_config, end_config):
		return numpy.linalg.norm(end_config - start_config)


	def Extend(self, start_config, end_config):
		epsilon = .01
		dist = self.ComputeDistance(start_config, end_config)
		numSteps = math.ceil(dist / epsilon)
		step = (end_config - start_config) / numSteps
		best_config = None
		i = 1
		while i <= numSteps:
			cur_config = start_config+step*i
			# Check if it is collision free
			with self.robot:
				robot_pos = self.robot.GetActiveDOFValues()
				robot_pos = cur_config
				self.robot.SetActiveDOFValues(robot_pos)
				## should we also check self.robot.CheckSelfCollision?
				if self.robot.GetEnv().CheckCollision(self.robot):
					return best_config
			#update variables
			best_config = cur_config
			i += 1
		return end_config
		
	def ShortenPath(self, path, timeout=5.0):
		#print('starting path shortening')
		start_time = time.time()
		current_time = 0
		i = 0
		totalDistance = 0
		#TODO: Calculate current path distance
		for i in range(0,len(path)-1):
			curDistance = self.ComputeDistance(path[i],path[i+1])
			totalDistance += curDistance
		print(totalDistance)
		i = 0
		while current_time < timeout and i == 0:
			#print("NOW IN THE FUNCTION LOOP")
			pathLength = len(path)
			p1index = random.randint(0,pathLength-2)
			p2index = p1index
			while p2index == p1index or abs(p2index-p1index) <= 1:
				p2index = random.randint(0,pathLength-2)
			if p1index > p2index:
				p1index, p2index = p2index,p1index
			p1a = numpy.array(path[p1index])
			p1b = numpy.array(path[p1index+1])
			p2a = numpy.array(path[p2index])
			p2b = numpy.array(path[p2index+1])  


			#choose random interpolation between points
			p1Vec = (p1b-p1a)
			p2Vec = (p2b-p2a)

			p1 = p1a + (p1Vec*random.random())
			p2 = p2a + (p2Vec*random.random())

			#TODO use extend to move to the two random points
			extender = self.Extend(p1,p2)
			if extender is not None:
				if self.ComputeDistance(extender,p2) < .01:
					#TODO get new path
					#print('shortening path now')
					badIndeces = [p1index+1,p2index]
					if (badIndeces[1]-badIndeces[0]) != 0:
						path[badIndeces[0]:badIndeces[1]] = []
					else:
						path[badIndeces[0]] = []
					path[badIndeces[0]] = p1
					path.insert(badIndeces[0]+1,p2)
					#print('new path created')

		# repeate until timeout
			current_time = time.time()-start_time
			i = 0
			
		totalDistance = 0
		#TODO: Calculate current path distance
		for i in range(0,len(path)-1):
			curDistance = self.ComputeDistance(path[i],path[i+1])
			totalDistance += curDistance
		print(totalDistance)    
		return path