import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.1):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        # TO DO: 
        FAR = True
        i = 1
        print goal_config
        while FAR:
            print "Iteration Number: " + str(i)
            i+=1

            # Sample a random configuration in Cfree
            sample = self.planning_env.GenerateRandomConfiguration()

            # Find the nearest(best) vertex and start extending from there
            best_id, best_v = tree.GetNearestVertex(sample)
            validSample = self.planning_env.Extend(best_v,sample)
            if validSample is None:
                pass
            else:
                validSample_id = tree.AddVertex(validSample)
                tree.AddEdge(best_id ,validSample_id)
                if self.visualize:
                    self.planning_env.PlotEdge(best_v, validSample)

                # Termination Condition
                if self.planning_env.ComputeDistance(validSample, goal_config) < epsilon:
                    goal_id = tree.AddVertex(goal_config)
                    tree.AddEdge(validSample_id, goal_id)
                    if self.visualize:
                        self.planning_env.PlotEdge(best_v, validSample)
                    FAR = False

        # Generate the path back from the goal
        curr_id = goal_id
        while curr_id != 0:
            plan.append(tree.vertices[curr_id])
            curr_id = tree.edges[curr_id]
        plan.append(start_config)
        plan.reverse()
        
        
        return plan