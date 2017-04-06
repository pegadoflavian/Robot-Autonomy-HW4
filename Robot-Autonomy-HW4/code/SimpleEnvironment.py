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
        self.lower_limits,self. upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

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

        config_control = []
        speed = 1.0
        duration = 0.4

        #get four possible controls

        #go straight
        config_control.append(Control(speed,speed,duration))
        #go back
        config_control.append(Control(-speed,-speed,duration))
        #turn right by pi/4 radians
        config_control.append(Control(speed,-speed,numpy.pi/4.0))
        #turn left by pi/4 radians
        config_control.append(Control(-speed,speed,numpy.pi/4.0))
              
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
            control_footprints = []
            current_actions = []

            #get the corresponding footprints for each control
            for c in config_control:
                control_footprints = self.GenerateFootprintFromControl(current_config,c)
                action = Action(c,control_footprints)
                self.actions[idx].append(action)
        print len(self.actions)

    def GetSuccessors(self, node_id):

        print "get succesors"
        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        grid_idx = self.discrete_env.NodeIdToGridCoord(node_id)
        current_config = self.discrete_env.NodeIdToConfiguration(node_id)
        current_config = numpy.asarray(current_config)

        orientation = grid_idx[2]

        curActions = self.actions[orientation]
        #print curActions

        for action in curActions:
            control = action.control
            collision = False

            for config in action.footprint:
                config_abs = numpy.array(config)
                config_abs[:2] += current_config[:2]

                if self.CheckCollision(config_abs) or not self.configInLimits(config_abs):
                    collision = True
                    print "collision"
                    break
            if not collision:
                final_config = numpy.array(action.footprint[-1][:])
                final_config[:2] +=  current_config[:2]
                succ_id = self.discrete_env.ConfigurationToNodeId(numpy.array(final_config))
                successors.append([succ_id,action])
                print "new successor"

            print "ok"
        return successors
        '''
        coordinate = self.discrete_env.NodeIdToGridCoord(node_id)
        start_config = self.discrete_env.NodeIdToConfiguration(node_id)
        idx = coordinate[2]
        action = self.actions[idx]
        print 'test'
        for act in action:
            flag= False
            id_list = []
            #add_foot_print
            for delta in act.footprint:
                tmp_ori=delta[2]
                cur_config = delta+ start_config
                cur_config[2] = tmp_ori
                id_list.append(self.discrete_env.ConfigurationToNodeId(cur_config))
            id_list = list(set(id_list))
                        
            for each_id in id_list:
                cur_config = self.discrete_env.NodeIdToConfiguration(each_id)
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
            cur_config = act.footprint[-1] + start_config
            cur_config[2] = delta[2]
            if node_id == self.discrete_env.ConfigurationToNodeId(cur_config):
                continue
            successors.append([self.discrete_env.ConfigurationToNodeId(cur_config), 
                          act])
        return successors
        '''
    def CheckCollision(self, point):
        T = self.robot.GetTransform()
        temp = T.copy()
        temp[0,3] = point[0]
        temp[1,3] = point[1]
        temp[0:3,0:3]=numpy.array([[numpy.cos(point[2]),-numpy.sin(point[2]),0],[numpy.sin(point[2]),numpy.cos(point[2]),0],[0,0,1]])
        with self.robot.GetEnv():
                self.robot.SetTransform(temp)
        return self.robot.GetEnv().CheckCollision(self.robot)

    def configInLimits(self, config):
        if (config>self.lower_limits).all() and (config<self.upper_limits).all():
            return True
        return False
    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
        goal_config = numpy.array(self.discrete_env.NodeIdToConfiguration(end_id))
        dist = numpy.linalg.norm(goal_config - start_config)
        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
        start_config=self.discrete_env.NodeIdToGridCoord(start_id)
        goal_config=self.discrete_env.NodeIdToGridCoord(goal_id)
        # for i in range(len(start_conf)):
        #     cost=cost+abs(start_conf[i]-goal_conf[i])
        cost = 10*(abs(start_config[0] - goal_config[0]) + abs(start_config[1] - goal_config[1]))
        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        
        return cost

    def boundary_check(self, config):
        coords = self.discrete_env.ConfigurationToGridCoord(config)
        if (numpy.all(coords >= numpy.array([0]*self.discrete_env.dimension))) and \
               (numpy.all(coords <  numpy.array(self.discrete_env.num_cells))):
            return True
        else:
            return False

    def robot_collision_check(self, config):

        tmp = config
        
        with self.robot.GetEnv():
            cfg = self.herb.GetCurrentConfiguration()
            self.herb.SetCurrentConfiguration(tmp)
            col_env= self.robot.GetEnv().CheckCollision(self.robot)
            col_self = self.robot.CheckSelfCollision()
        if (col_env or col_self):
            return True
        return False

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

    def PlotPlan(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'r.-', linewidth=2.5)
        pl.draw()
