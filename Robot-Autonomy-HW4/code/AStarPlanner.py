import time
import bisect
import numpy as np
#from SimpleEnvironment import Action
class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.open = Openlist(planning_env)
        self.close = Closelist(planning_env)


    def Plan(self, start_config, goal_config):
        start_time = time.time()

        plan = []
        node_control_mapping = dict()

        start_node = Node(0,None,0,self.planning_env.discrete_env.ConfigurationToNodeId(start_config),None)
        self.open.addNode(start_node)
        
        self.planning_env.InitializePlot(goal_config)

        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        self.open.setGoal(goal_id)
        goal_node = Node(0,None,0,goal_id,None)
        suc_node = start_node
        self.close.addNode(start_node.id)
        count = 0
        prev_dist_to_goal = 1000
        flag_almost_there  =False
        while (self.open.isEmpty() == False):
            #import IPython
            #IPython.embed()
            curr = self.open.getlowest()
            dist_to_goal = self.planning_env.ComputeDistance(curr.id,goal_id)
            print dist_to_goal
            if(curr.parent != None):
                cur_config = np.array(self.planning_env.discrete_env.NodeIdToConfiguration(curr.id))
                parent_config = np.array(self.planning_env.discrete_env.NodeIdToConfiguration(curr.parent.id))
                self.planning_env.PlotEdge(cur_config,parent_config)
            count = count + 1
            #curr = self.open.getlowest()
            if not flag_almost_there and (dist_to_goal == 0.0):
                flag_almost_there = True
            if (curr.id == goal_id):# or (dist_to_goal == 0.0):
                suc_node = curr
                print 'bye bye'
                break
            elif flag_almost_there and (dist_to_goal > 0.0):
                print "error increased"
                suc_node = curr
                break
            prev_dist_to_goal = dist_to_goal
            successors = self.planning_env.GetSuccessors(curr.id)

            for i in range(0, len(successors)):
                if (self.close.isDuplicate(successors[i][0]) == False):
                    #print 'got here'
                    newnode = Node(curr.cost+successors[i][1].control.dt,curr,curr.depth+1, successors[i][0],successors[i][1])#Action(successors[i][1].control,successors[i][1].footprint))
                    self.open.addNode(newnode)
                    self.close.addNode(successors[i][0])
            print self.open.isEmpty()

        print suc_node.id, start_node.id

        while (suc_node.id != start_node.id):
            plan.insert(0,suc_node.action)
            suc_node = suc_node.parent
            print "inserted node"
        #plan.insert(0,Action([0,0,0],np.asarray(start_config)))
        print("--- %s seconds ---" % (time.time() - start_time))
        print("--- %d expansions ---" % count)
        transform = self.planning_env.robot.GetTransform() #hacky
        transform[0, 3] = start_config[0]
        transform[1, 3] = start_config[1]
        self.planning_env.robot.SetTransform(transform)
        #plan.append(Action(node_control_mapping[str(self.planning_env.discrete_env.ConfigurationToNodeId(goal_config))],goal_config))
        #plan.append(Action([0,0,0],np.asarray([goal_config])))
        print plan
        return plan


class Node:
    def __init__(self, cost, parent, depth, id, action):
        self.cost = cost
        self.parent = parent
        self.depth = depth
        self.id = id
        self.action = action

    def __str__(self):
        #Spits out a full list the node's state/operator and states/operators leading to it
        result = "Cost: " +  str(self.cost)
        result += " Depth: " + str(self.depth)
        result += " ID: " + str(self.id)
        if self.parent != None:
            result += " Parent: " + str(self.parent.id)
        return result


class Closelist:

    def __init__(self, env):
        #import IPython
        #IPython.embed()
        self.close = []

    def addNode(self, id):
        bisect.insort(self.close,id)

    def isDuplicate(self, id):
        #import IPython
        #IPython.embed()
        if(bisect.bisect_left(self.close,id) == len(self.close)):
            return False
        if (self.close[bisect.bisect_left(self.close,id)] == id):
            return True
        return False


class Openlist:
    def __init__(self, env):
        self.env = env
        self.open = []
        self.goal_id = None
    def __str__(self):
        result = "Open List contains " + str(len(self.open)) + " items\n"
        for item in self.open:
            result += str(item) + "\n"
        return result
    def setGoal(self, goal_id):
        self.goal_id = goal_id

    def addNode(self, node):
        lowest = 0
        greatest = len(self.open)
        oldmid = -1
        midpoint = 0
        weight = 10
        while(lowest < greatest):
            midpoint = (lowest + greatest)/2
            if(self.open[midpoint].cost+weight*self.env.ComputeHeuristicCost(self.open[midpoint].id, self.goal_id) == node.cost+weight*self.env.ComputeHeuristicCost(node.id, self.goal_id) or oldmid == midpoint):
                lowest = midpoint
                break;
            else:
                if (node.cost+weight*self.env.ComputeHeuristicCost(node.id, self.goal_id) < self.open[midpoint].cost+weight*self.env.ComputeHeuristicCost(self.open[midpoint].id, self.goal_id)):
                    greatest = midpoint
                else:
                    lowest = midpoint+1
            oldmid = midpoint
        self.open.insert(lowest, node)

    def getlowest(self):
        if not self.isEmpty():
            return self.open.pop(0)
        else:
            raise RunTimeError

    def isEmpty(self):
        return len(self.open) == 0