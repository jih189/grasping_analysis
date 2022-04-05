#!/usr/bin/python
from glob import glob
import os
# this script will build the DMG tabe
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import *
from ffregrasp import ff_regrasp_planner
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
    base = pandactrl.World(camp=[700,700,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    # objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")
    # objpath = os.path.join(this_dir, "objects", "bottle.stl")
    # objpath = os.path.join(this_dir, "objects", "can.stl")
    # objpath = os.path.join(this_dir, "objects", "Ushape.stl")
    objpath = os.path.join(this_dir, "objects", "Hshape.stl")
    dbobjname = os.path.splitext(os.path.basename(objpath))[0]
    print("object name ", dbobjname)

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
    handname = handpkg.getHandName()
    print("hand name ", handname)
    fetchhnd = handpkg.newHandNM(hndcolor=[0,1,0,1])
    # test
    fetchhnd.setJawwidth(30)
    fetchhnd.setMat(pandanpmat4=pg.cvtMat4np4(np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])))
    fetchhnd.reparentTo(base.render)

    gdb = db.GraspDB()

    freeairgripdata = gdb.loadFreeAirGrip(dbobjname, handname = handname)
    if freeairgripdata is None:
        raise ValueError("Plan the freeairgrip first!")

    freegripid = freeairgripdata[0]
    freegriprotmats = freeairgripdata[3]
    freegripjawwidth = freeairgripdata[4]

    currentGraspid = 0

    print("w,s: switch gras pose")

    def eventW():
        global currentGraspid
        if currentGraspid != 0:
            currentGraspid -= 1
        print(currentGraspid)
        fetchhnd.setMat(pandanpmat4=freegriprotmats[currentGraspid])

    def eventS():
        global currentGraspid
        if currentGraspid != len(freegripid):
            currentGraspid += 1
        print(currentGraspid)
        fetchhnd.setMat(pandanpmat4=freegriprotmats[currentGraspid])


    base.accept('w', eventW)
    base.accept('w-repeat', eventW)

    base.accept('s', eventS)
    base.accept('s-repeat', eventS)

    base.run()
