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
        '''
        grid=self.discrete_env.NodeIdToGridCoord(node_id)
        
        for i in range(0,self.discrete_env.dimension):
            #newCoord=grid[:]
            print "newCoord"
            print grid

            # Surbtaction neighbor
            grid[i] = grid[i]+1
            print "g1"
            print grid
            newNode = self.discrete_env.GridCoordToNodeId(grid)
            print "node"
            print newNode
            if (self.IsCollision(newNode )==False  and self.IsBoundary(newNode)==True):
                successors.append(newNode )

            # Addition Neighbor
            grid[i] = grid[i]-2
            print "g2"
            print grid
            print "node"
            print newNode
            newNode  = self.discrete_env.GridCoordToNodeId(grid)
            if (not self.IsCollision(newNode)  and self.IsBoundary(newNode)):
                successors.append(newNode )
            grid[i]=grid[i]+1
        
        grid_idx = self.discrete_env.NodeIdToGridCoord(node_id)
        current_config = self.discrete_env.NodeIdToConfiguration(node_id)
        current_config = numpy.asarray(current_config)

        orientation = grid_idx[2]

        curAction = self.actions[orientation]
        '''
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

