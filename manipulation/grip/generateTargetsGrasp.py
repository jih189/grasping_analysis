#!/usr/bin/python
import os
# this script will build the DMG tabe
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import *
from ffregrasp import ff_regrasp_planner, PandaPosMax_t_PosMat, PosMat_t_PandaPosMax
from manipulation.grip.fetch_gripper import fetch_grippernm
from database import dbaccess as db
from utils import dbcvt as dc
import pandaplotutils.pandactrl as pandactrl
import trimesh
from pandaplotutils import pandageom as pg
import numpy as np
from direct.task import Task
import math


# class command_base:
#     def __init__(self, handpkg, base):
#         # load model
        
#         fetchhnd = handpkg.newHandNM(hndcolor=[0,1,0,1])
#         # test
#         fetchhnd.setJawwidth(10)
#         fetchhnd.reparentTo(base.render)

#     def SetupEvents(self):


if __name__ == '__main__':
    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")

    gdb = db.GraspDB()
    objectId = None

    sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % os.path.splitext(os.path.basename(objpath))[0]
    result = gdb.execute(sql)
    if not result:
        print "please add the object name to table first!!!"
        exit()
    else:
        objectId = int(result[0][0]) 

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
    star.reparentTo(base.render)

    handpkg = fetch_grippernm
    fetchhnd = handpkg.newHandNM(hndcolor=[0,1,0,1])
    # test
    fetchhnd.setJawwidth(90)
    fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(np.array([[1,0,0,0.02], [0,1,0,0], [0,0,1,0], [0,0,0,1]])))
    fetchhnd.reparentTo(base.render)

    print "w,s,a,d,q,e: translation"
    print "8,2,4,6,7,9: rotation"
    print "z: open gripper"
    print "c: close gripper"
    print "x: print pose"

    def shift(pose):
        return pose.dot(np.array([[1,0,0,-0.02], [0,1,0,0], [0,0,1,0], [0,0,0,1]]))

    def shiftback(pose):
        return pose.dot(np.array([[1,0,0,0.02], [0,1,0,0], [0,0,1,0], [0,0,0,1]]))

    def eventW():
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,-0.005], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    def eventS():
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0.005], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    def eventA():
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0], [0,1,0,-0.005], [0,0,1,0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    def eventD():
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0], [0,1,0,0.005], [0,0,1,0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    def eventQ():
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0.005], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    def eventE():
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,-0.005], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('w', eventW)
    base.accept('w-repeat', eventW)

    base.accept('s', eventS)
    base.accept('s-repeat', eventS)

    base.accept('a', eventA)
    base.accept('a-repeat', eventA)

    base.accept('d', eventD)
    base.accept('d-repeat', eventD)

    base.accept('q', eventQ)
    base.accept('q-repeat', eventQ)

    base.accept('e', eventE)
    base.accept('e-repeat', eventE)

    def event8():
        rotateStep = math.pi / 36
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[math.cos(rotateStep),0,math.sin(rotateStep),0], [0,1,0,0], [-math.sin(rotateStep),0,math.cos(rotateStep),0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('8', event8)
    base.accept('8-repeat', event8)

    def event2():
        rotateStep = -math.pi / 36
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[math.cos(rotateStep),0,math.sin(rotateStep),0], [0,1,0,0], [-math.sin(rotateStep),0,math.cos(rotateStep),0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('2', event2)
    base.accept('2-repeat', event2)

    def event4():
        rotateStep = math.pi / 36
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0], [0,math.cos(rotateStep),-math.sin(rotateStep),0], [0,math.sin(rotateStep),math.cos(rotateStep),0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('4', event4)
    base.accept('4-repeat', event4)

    def event6():
        rotateStep = -math.pi / 36
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[1,0,0,0], [0,math.cos(rotateStep),-math.sin(rotateStep),0], [0,math.sin(rotateStep),math.cos(rotateStep),0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('6', event6)
    base.accept('6-repeat', event6)

    def event7():
        rotateStep = math.pi / 36
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[math.cos(rotateStep),-math.sin(rotateStep),0,0], [math.sin(rotateStep),math.cos(rotateStep),0,0], [0,0,1,0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('7', event7)
    base.accept('7-repeat', event7)

    def event9():
        rotateStep = -math.pi / 36
        currentPose = shift(PandaPosMax_t_PosMat(fetchhnd.getMat()))
        moveupmat = np.array([[math.cos(rotateStep),-math.sin(rotateStep),0,0], [math.sin(rotateStep),math.cos(rotateStep),0,0], [0,0,1,0], [0,0,0,1]])
        currentPose = shiftback(currentPose.dot(moveupmat))
        fetchhnd.setMat(pandanpmat4=PosMat_t_PandaPosMax(currentPose))

    base.accept('9', event9)
    base.accept('9-repeat', event9)

    def eventZ():
        currentWidth = fetchhnd.jawwidth
        currentWidth -= 5
        if currentWidth >= 0:
            fetchhnd.setJawwidth(currentWidth)

    base.accept('z', eventZ)
    base.accept('z-repeat', eventZ)

    def eventC():
        currentWidth = fetchhnd.jawwidth
        currentWidth += 5
        if currentWidth <= 90:
            fetchhnd.setJawwidth(currentWidth)

    base.accept('c', eventC)
    base.accept('c-repeat', eventC)

    def eventX():
        print dc.mat4ToStr(fetchhnd.getMat())
        print str(fetchhnd.jawwidth)

        sql = "INSERT INTO targetgrasps(idobject, grasppose, jawwidth) \
                                VALUES('%d', '%s', '%s')" % \
                                (objectId, dc.mat4ToStr(fetchhnd.getMat()), str(fetchhnd.jawwidth))
        gdb.execute(sql)

    base.accept('x', eventX)
    base.accept('x-repeat', eventX)

    base.run()
