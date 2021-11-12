#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
import itertools
from utils import dbcvt as dc
import networkx as nx

from database import dbaccess as db
import matplotlib.pyplot as plt
from fetch_gripper import fetch_grippernm
from utils import collisiondetection as cd
from regrasp_planner import RegripPlanner

class FF_replacement_planner:
    def __init__(self, planner):
        self.planner = planner
        self.freegripid = planner.freegripid
        self.placementid = planner.placementid
        self.placementtype = planner.placementtype
        self.tpsmat4s = planner.tpsmat4s

        self.G = nx.Graph()
        self. __CreatePlacementGraph()
        # path of inti placement to target placement alogn with stablity of placement: [(p1,s),(p2,u)]
        self.path_and_type = [] 
        # List of all grap id for path of init placement to target placement: [ [gi_p1_p2, gi1_p1_p2,..], [gi_p2_p3,...]]
        self.grasp_trajectory = []


    def __CreatePlacementGraph(self):
        for p in range(len(self.placementid)):
            self.G.add_node(self.placementid[p], stable=self.placementtype[p])
        
        for i in range(len(self.freegripid)):
            sql = "SELECT freetabletopgrip.idfreetabletopplacement FROM freetabletopgrip WHERE \
                freetabletopgrip.idfreeairgrip=%d" % self.freegripid[i]
            result = self.gdb.execute(sql)
            if len(result) > 1:
                for edge in list(itertools.combinations(np.array(result)[:,0], 2)):
                    if not self.G.has_edge(*edge):
                        self.G.add_edge(*edge, graspid=[self.freegripid[i]])
                    else:
                        temp = self.G.edges[edge[0],edge[1]]['graspid']
                        temp.append(self.freegripid[i])
                        self.G.add_edge(*edge, graspid=temp)

    def insert_int_graspID_as_placementGraphNode(self, graspid):
        self.inital_grasp = ("int_g", graspid)
        self.G.add_node("int_g",stable=-1)
        placement_list = self.planner.getPlacementIdsByGraspId(graspid)
        for p in placement_list:
            edge = ("int_g",p)
            self.G.add_edge(*edge)
        
    def insert_end_graspID_as_placementGraphNode(self,graspid):
        self.end_grasp = ("end_g", graspid)
        self.G.add_node("end_g", stable=-1)
        placement_list = self.planner.getPlacementIdsByGraspId(graspid)
        for p in placement_list:
            edge = ("end_g",p)
            self.G.add_edge(*edge)

    def insert_end_graspPose_as_placementGraphNode(self, goalrotmat4, goalhandwidth):
        self.end_grasp = ("end_g", -1)
        self.G.add_node("end_g", stable=-1)
        
        # # check which placement this grasp belong to
        for p in range(len(self.placementid)):
            
            # if the hand does not hit the ground, then this placement can connect to the goal node
            tmphnd = self.planner.hand
            tmphnd.setJawwidth(goalhandwidth)
            tmphnd.setMat(pandanpmat4 = goalrotmat4 * self.tpsmat4s[p])
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                self.regg.add_edge('end_g', self.placementid[p])        

    def find_shortest_grasp_path(self):
        path = nx.shortest_path(self.G,self.inital_grasp[0], self.end_grasp[0])
        #remove end graps node and init grasp node in graph path.
        path.pop()
        path.pop(0)
        for p in path:
            type = self.G.nodes[p]["stable"]
            self.path_and_type.append((p,type))
        return self.path_and_type

    def get_placement_grasp_trajectory(self):
        path = self.path_and_type
        if len(path) > 1:
            for i in range(0,len(path)-1):
                currnt_p = path[i][0]
                next_p = path[i+1][0]
                graspid_list = self.G.get_edge_data(currnt_p,next_p)['graspid']
                self.grasp_trajectory.append(graspid_list)
        elif len(path) == 1:
            return []
        else:
            print("error, no Placement path be grasp points")
        return self.grasp_trajectory



if __name__ == '__main__':
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "objects", "cup.stl")
    handpkg = fetch_grippernm
    gdb = db.GraspDB()

    planner = RegripPlanner(objpath, handpkg, gdb)
    replacement_planner = FF_replacement_planner(planner)

    placementId1 = 1
    placementID2 = 3
    
    startGraspId = 55
    goalGraspId = 25#30, 180
    replacement_planner.insert_int_graspID_as_placementGraphNode(startGraspId)
    replacement_planner.insert_end_graspID_as_placementGraphNode(goalGraspId)

    path = replacement_planner.find_shortest_grasp_path()
    grasp_t = replacement_planner.get_placement_grasp_trajectory()
    
    print(path)
    print(grasp_t)
   
    g = replacement_planner.G 
    #print(g.nodes)
    #print(g.edges.data())
    labels = {n: g.nodes[n]['stable'] for n in g.nodes}
    colors = [g.nodes[n]['stable'] for n in g.nodes]
    nx.draw(g, with_labels=True, labels=labels, node_color=colors)
    plt.draw()
    plt.show()
