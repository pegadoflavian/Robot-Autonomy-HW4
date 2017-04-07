import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        # max edge length + a lot of code clean up
        maxDist = .5
        michaelsi = 1
        while True:
            sample = self.planning_env.GenerateRandomConfiguration()
            f_vid, f_vtx = ftree.GetNearestVertex(sample)
            r_vid, r_vtx = rtree.GetNearestVertex(sample)
            f_vtx_sample_dist = self.planning_env.ComputeDistance(sample,f_vtx)
            r_vtx_sample_dist = self.planning_env.ComputeDistance(sample,r_vtx)
            f_sample = f_vtx + (sample-f_vtx)/f_vtx_sample_dist * min(f_vtx_sample_dist, maxDist)
            r_sample = r_vtx + (sample-r_vtx)/r_vtx_sample_dist * min(r_vtx_sample_dist, maxDist)
            f_best = self.planning_env.Extend(f_vtx, f_sample)
            r_best = self.planning_env.Extend(r_vtx, r_sample)
            print michaelsi
            michaelsi += 1

            if f_best is not None:
                f_new_vid = self.AddNode(ftree, f_vid, f_best)
                # check if new vertex I'm adding joins the trees
                connects, r_close_vid = self.CheckIfOtherTreeConnects(f_best, rtree, epsilon)
                if(connects):
                    return self.GeneratePlan(ftree, rtree, f_new_vid, r_close_vid)

            if r_best is not None:
                r_new_vid = self.AddNode(rtree, r_vid, r_best)
                connects, f_close_vid = self.CheckIfOtherTreeConnects(r_best, ftree, epsilon)
                if(connects):
                    return self.GeneratePlan(ftree, rtree, f_close_vid, r_new_vid)

    def AddNode(self, tree, parent_id, cfg):
        new_id = tree.AddVertex(cfg)
        tree.AddEdge(parent_id, new_id)
        if(self.visualize):
            self.planning_env.PlotEdge(tree.vertices[parent_id], cfg)
        self.planning_env.robot.SetActiveDOFValues(cfg)
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
