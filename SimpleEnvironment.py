import numpy, openravepy,pdb
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        print "construct actions"
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)
            
            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            current_config = numpy.asarray(start_config).copy()
            config_control = []
            control_footprints = []
            current_actions = []
            speed = 1.0
            duration = 0.5

            #get four possible controls

            #go straight
            config_control.append(Control(speed,speed,duration))
            #go back
            config_control.append(Control(-speed,-speed,duration))
            #turn right by pi/4 radians
            config_control.append(Control(speed,-speed,numpy.pi/4.0))
            #turn left by pi/4 radians
            config_control.append(Control(-speed,speed,numpy.pi/4.0))

            #get the corresponding footprints for each control
            for c in config_control:
                control_footprints.append(self.GenerateFootprintFromControl(current_config,c))

            #create the corresponding actions
            for i in range(len(config_control)):
                current_actions.append(Action(config_control[i],control_footprints[i]))


            self.actions[idx] = current_actions
            #pdb.set_trace()
        print self.actions

    def GetSuccessors(self, node_id):

        print "get succesors"
        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        coordinate = self.discrete_env.NodeIdToGridCoord(node_id)
        start_config = self.discrete_env.NodeIdToConfiguration(node_id)
        idx = coordinate[2]
        action = self.actions[idx]
        print 'test'
        for act in action:
            flag= False
            id_list = []
            #add_foot_print
            for delta in action.footprint:
                tmp_ori=delta[2]
                cur_config= delta.copy()+ start_config.copy()
                cur_config[2] = tmp_ori
                id_list.append(self.discrete_env.ConfigurationToNodeId(cur_config))
            id_list = list(set(id_list))
                        
            for each_id in id_list:
                cur_config = self.discrete_env.NodeIdToConfiguration(each_id).copy()
                flag_1 = self.robot_collision_check(cur_config)
                #check collision
                if flag_1==True:
                    flag=True
                    break
                #check whether the robot is in the boundary
                flag_2 = self.boundary_check(cur_config)
                if flag_2==False:
                    flag=True
                    break
            if flag==True:
                continue
            cur_config = act.footprint[-1].copy() + start_config.copy()
            cur_config[2] = delta[2].copy()
            if node_id == self.discrete_env.ConfigurationToNodeId(xconfig):
                continue
            successors.append([self.discrete_env.ConfigurationToNodeId(cur_config.copy()), 
                          act])
        return successors
        

    def ComputeDistance(self, start_id, end_id):

        dist = 0
        line = NodeIdToConfiguration(end_id) - NodeIdToConfiguration(start_id)
        dist = numpy.linalg.norm(line)
        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        start_conf=self.discrete_env.NodeIdToGridCoord(start_id)
        goal_conf=self.discrete_env.NodeIdToGridCoord(goal_id)
        for i in range(len(start_conf)):
            cost=cost+abs(start_conf[i]-goal_conf[i])
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost
    def boundary_check(self, config):
        coords = self.discrete_env.ConfigurationToGridCoord(config)
        if (np.all(coords >= np.array([0]*self.discrete_env.dimension))) and \
               (np.all(coords <  np.array(self.discrete_env.num_cells))):
            return True
        else:
            return False
    def robot_collision_check(self, config):

        tmp = config.copy()
        
        with self.robot.GetEnv():
            cfg = self.herb.GetCurrentConfiguration()
            self.herb.SetCurrentConfiguration(tmp)
            col_env= self.robot.GetEnv().CheckCollision(self.robot)
            col_self = self.robot.CheckSelfCollision()
        if (col_env or col_self):
            return True
        return False

