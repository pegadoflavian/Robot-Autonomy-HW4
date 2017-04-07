import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        # max edge length + a lot of code clean up
        maxDist = 5
        milestone = 10
        while True:
            sample = self.planning_env.GenerateRandomConfiguration()
            f_vid, f_best = self.ExtendFromTree(ftree,sample,maxDist, epsilon)
            r_vid, r_best = self.ExtendFromTree(rtree,sample,maxDist, epsilon)
            if f_best is not None:
                f_ids = self.AddNodeAndMilestones(ftree, f_vid, f_best, milestone)
                for id in f_ids:
                    # check if new vertex I'm adding joins the trees
                    connects, r_close_vid = self.CheckIfOtherTreeConnects(ftree.vertices[id], rtree, epsilon)
                    if(connects):
                        return self.GeneratePlan(ftree, rtree, id, r_close_vid)

            if r_best is not None:
                r_ids = self.AddNodeAndMilestones(rtree, r_vid, r_best, milestone)
                for id in r_ids:
                    # check if new vertex I'm adding joins the trees
                    connects, f_close_vid = self.CheckIfOtherTreeConnects(rtree.vertices[id], ftree, epsilon)
                    if(connects):
                        return self.GeneratePlan(ftree, rtree, f_close_vid, id)

    def ExtendFromTree(self, tree, sample, maxDist,epsilon):
        vid, vtx = tree.GetNearestVertex(sample)
        vtx_sample_dist = self.planning_env.ComputeDistance(sample, vtx)
        if(vtx_sample_dist >= epsilon):
            sample = vtx + (sample - vtx)/vtx_sample_dist * min(vtx_sample_dist, maxDist)
            return (vid, self.planning_env.Extend(vtx, sample))
        return (None, None)

    def AddNodeAndMilestones(self, tree, parent_id, cfg, milestone):
        start = tree.vertices[parent_id]
        dist = self.planning_env.ComputeDistance(start,cfg)
        direction = (cfg - start)/dist
        accum = milestone
        ids = []
        while accum < dist:
            parent_id = self.AddNode(tree, parent_id, start+direction*accum)
            ids.append(parent_id)
            accum += milestone
        ids.append(self.AddNode(tree, parent_id, cfg))
        return ids

    def AddNode(self, tree, parent_id, cfg):
        new_id = tree.AddVertex(cfg)
        tree.AddEdge(parent_id, new_id)
        if(self.visualize):
            self.planning_env.PlotEdge(tree.vertices[parent_id], cfg)
        self.planning_env.robot.SetActiveDOFValues(cfg)
        print(cfg)
        return new_id

    def CheckIfOtherTreeConnects(self, testCfg, otherTree, epsilon):
        vid, vtx = otherTree.GetNearestVertex(testCfg)
        if(self.planning_env.ComputeDistance(testCfg, vtx) < epsilon):
            return (True, vid)
        return (False, vid)

    def GeneratePlan(self, ftree, rtree, f_vid, r_vid):
        # build plan here
        plan = []
        cursor = f_vid
        while(cursor != ftree.GetRootId()):
            plan.append(ftree.vertices[cursor])
            cursor = ftree.edges[cursor]
        plan.append(ftree.vertices[cursor])
        plan.reverse()
        cursor = r_vid
        while(cursor != rtree.GetRootId()):
            cursor = rtree.edges[cursor]
            plan.append(rtree.vertices[cursor])
        return plan
