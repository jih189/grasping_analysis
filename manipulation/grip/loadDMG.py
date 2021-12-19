#!/usr/bin/python
import os
# this script will build the DMG tabe
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import *
from ffregrasp import ff_regrasp_planner
from manipulation.grip.fetch_gripper import fetch_grippernm
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
import trimesh
from pandaplotutils import pandageom as pg
import numpy as np
from direct.task import Task

if __name__ == '__main__':
    base = pandactrl.World(camp=[1000,700,0], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "objects", "book.stl")
    handpkg = fetch_grippernm
    gdb = db.GraspDB()
    regrasp_planner = ff_regrasp_planner(objpath, handpkg, gdb)

    regrasp_planner.loadDB()

    placementId = 2
    startGraspId = 2
    goalGraspId = 40

    startPose_panda, startJawwidth_panda = regrasp_planner.getGrasp(placementId, startGraspId)
    goalPose_panda, goalJawwidth_panda = regrasp_planner.getGrasp(placementId, goalGraspId)
    placementPose = regrasp_planner.getPlacement(placementId)
    inv_placementPose = np.linalg.inv(pg.mat4ToNp(placementPose))
    
    # pass the init grasp and target grasp in object frame
    # pass the object placement in the table frame
    grasp_trajectory = regrasp_planner.getTrajectory(inv_placementPose.dot(pg.mat4ToNp(startPose_panda)), 
                inv_placementPose.dot(pg.mat4ToNp(goalPose_panda)), 80, pg.mat4ToNp(placementPose), base)

    if grasp_trajectory == None:
        print("no path between init grasp to target grasp")
        exit()

    poseTrajectory = []
    print("len of grasp = ", len(grasp_trajectory))
    for g in range(len(grasp_trajectory) - 1):
        trajectory = regrasp_planner.getLinearPoseTrajectory(pg.mat4ToNp(placementPose).dot(grasp_trajectory[g]), pg.mat4ToNp(placementPose).dot(grasp_trajectory[g+1]))
        poseTrajectory.extend(trajectory)


    ################ following code is used to demo ####################################################
    # poseTrajectory: a grasp pose trajectory from start pose to goal pose
    # create a hand for demo
    starthnd = regrasp_planner.handpkg.newHandNM(hndcolor=[0, 0, 1, 0.5])
    starthnd.setMat(pandanpmat4 = startPose_panda)
    starthnd.setJawwidth(startJawwidth_panda)
    starthnd.reparentTo(base.render)

    goalhnd = regrasp_planner.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
    goalhnd.setMat(pandanpmat4 = goalPose_panda)
    goalhnd.setJawwidth(goalJawwidth_panda)
    goalhnd.reparentTo(base.render)

    objtrimesh=trimesh.load_mesh(objpath)
    geom = pg.packpandageom(objtrimesh.vertices,
                            objtrimesh.face_normals,
                            objtrimesh.faces)

    node = GeomNode('obj')
    node.addGeom(geom)
    star = NodePath('obj')
    star.attachNewNode(node)
    star.setColor(Vec4(.7,0.3,0,1))
    star.setTransparency(TransparencyAttrib.MAlpha)
    star.setMat(placementPose)
    star.reparentTo(base.render)
    
    currenthnd = regrasp_planner.handpkg.newHandNM(hndcolor=[0, 0.5, 0.5, 0.5])
    currenthnd.setJawwidth(startJawwidth_panda)
    currenthnd.reparentTo(base.render)

    counter = 0
    def myFunction(task, poseTrajectory):
        global counter
        if counter > len(poseTrajectory) - 1:
            return task.done

        currenthnd.setMat(pandanpmat4 = pg.cvtMat4np4(poseTrajectory[counter]))
        counter += 1
        return task.again

    # show the collision net
    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont

    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    bullcldrnp = base.render.attachNewNode("bulletcollider")
    debugNP = bullcldrnp.attachNewNode(debugNode)
    debugNP.show()
    regrasp_planner.bulletworldhp.setDebugNode(debugNP.node())
    taskMgr.add(updateworld, "updateworld", extraArgs=[regrasp_planner.bulletworldhp], appendTask=True)

    myTask = taskMgr.doMethodLater(0.1, myFunction, 'tickTask', extraArgs=[Task.Task(myFunction), poseTrajectory])

    base.run()