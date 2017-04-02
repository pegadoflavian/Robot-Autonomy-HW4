#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy

# OpenRAVE
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
    self.openrave_init()
    self.problem_init()

    #order grasps based on your own scoring metric
    self.order_grasps()

    #order grasps with noise
    #self.order_grasps_noisy()


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
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
    # self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    # self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

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
    print "Ordering Ideal Grasps"
    self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
    for grasp in self.grasps_ordered:
      grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
    
    # sort!
    order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered = self.grasps_ordered[order]

  
  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp 
  def order_grasps_noisy(self):
    print "Ordering Noisy Grasps"
    self.grasps_ordered_noisy = self.grasps_ordered.copy() #you should change the order of self.grasps_ordered_noisy
    #TODO set the score with your evaluation function (over random samples) and sort
    import numpy
    num_samples = 5
    for grasp in self.grasps_ordered_noisy:
      true_score = self.eval_grasp(grasp)
      noisy_score = 0
      print("start")
      for i in range(num_samples):
        noisy_grasp = self.sample_random_grasp(grasp)
        noisy_score += self.eval_grasp(noisy_grasp)
        print(self.eval_grasp(noisy_grasp))
      noisy_score = noisy_score/num_samples
      grasp[self.graspindices.get('performance')] = noisy_score 
    
    # sort!
    order = np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered_noisy = self.grasps_ordered_noisy[order]    


  # function to evaluate grasps
  # returns a score, which is some metric of the grasp
  # higher score should be a better grasp
  def eval_grasp(self, grasp):
    with self.robot:
      #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
      try:
        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)  # Set this to true!

        obj_position = self.gmodel.target.GetTransform()[0:3,3]
        
        # for each contact
        G = np.zeros(shape = (len(contacts),6))
        i = 0
        for c in contacts:
          pos = c[0:3] - obj_position
          dirn = -c[3:] #this is already a unit vector
          
          #TODO fill G
          wrench = np.cross(pos,dirn)
          G[i,:] = np.array([np.concatenate((dirn,wrench))])
         
          i = i + 1
          
        G = G.transpose()

        u,s,v = np.linalg.svd(G)
        ratio = s.min()/s.max()
        # print ratio

        product = math.sqrt(max(np.linalg.det(np.dot(G,G.transpose())),0))
        
        return ratio

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
    # Make a copy of the grasp to add noise
    grasp = grasp_in.copy()

    # Sample random position
    pos_orig = grasp[self.graspindices['igrasppos']]
    RAND_DIST_MU = pos_orig  # Mean of Guassian = original position
    RAND_DIST_SIGMA = 0.0001 # Standard Deviation of Guassian
    
    # Add noise to position
    noisy_pos = np.random.normal(RAND_DIST_MU, RAND_DIST_SIGMA,3)


    #sample random orientation
    
    dir_orig = grasp[self.graspindices['igraspdir']]
    roll_orig = grasp[self.graspindices['igrasproll']]
    #TODO set the direction and roll to be random

    RAND_DIR_MU = dir_orig
    RAND_DIR_SIGMA = 0.01 #TODO you may want to change this
    noisy_dir = np.random.normal(RAND_DIR_MU, RAND_DIR_SIGMA,3)

    RAND_ANGLE_MU = roll_orig
    RAND_ANGLE_SIGMA = np.pi/45
    noisy_ang = np.random.normal(RAND_ANGLE_MU, RAND_ANGLE_SIGMA)

    grasp[self.graspindices['igrasppos']] = noisy_pos
    grasp[self.graspindices['igraspdir']] = noisy_dir
    grasp[self.graspindices['igrasproll']] = noisy_ang

    return grasp


  #displays the grasp
  def show_grasp(self, grasp, delay=1.5):
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

if __name__ == '__main__':
  
  robo = RoboHandler()
  import IPython
  IPython.embed()
  #time.sleep(10000) #to keep the openrave window open