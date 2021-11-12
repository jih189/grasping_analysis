#!/usr/bin/python
import os
# this script will build the DMG tabe
from ffregrasp import ff_regrasp_planner
from manipulation.grip.fetch_gripper import fetch_grippernm
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl

if __name__ == '__main__':

    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")

    handpkg = fetch_grippernm
    gdb = db.GraspDB()
    regrasp_planner = ff_regrasp_planner(objpath, handpkg, gdb)
    regrasp_planner.build_regrasp_graph_for_all_placements(base)
    for placementid in range(len(regrasp_planner.regrasp_graph)):
        print "for placement ", placementid, ", the number of plane is ", len(regrasp_planner.regrasp_graph[placementid])
    # print regrasp_planner.regrasp_graph[2][0][1].getEdges()
    regrasp_planner.saveToDB()
