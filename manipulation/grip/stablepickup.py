#!/user/bin/python

import os
import numpy as np

from manipulation.grip.fetch_gripper import fetch_grippernm
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import *
from shapely.geometry import LinearRing
from shapely.geometry import Point
from shapely.geometry import Polygon

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh
from trimesh import geometry as trigeom
from pandaplotutils import pandageom as pg
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from database import dbaccess as db

import random

class StablePickupPlanner(object):
    def __init__(self, objpath, handpkg, gdb):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.objcom = self.objtrimesh.center_mass
        self.objtrimeshconv=self.objtrimesh.convex_hull
        # oc means object convex
        self.ocfacets, self.ocfacetnormals = self.objtrimeshconv.facets_over(.9999)
        # ocfacets is the group of index of facets to be a face

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # use two bulletworld, one for the ray, the other for the tabletop
        self.bulletworldray = BulletWorld()
        self.bulletworldhp = BulletWorld()

        # plane to remove hand
        self.planebullnode = cd.genCollisionPlane(offset=0)
        self.bulletworldhp.attachRigidBody(self.planebullnode)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.7])

        self.counter = 0

        self.gdb = gdb
        self.loadFreeAirGrip()

        # plot the object
        geom = pg.packpandageom(self.objtrimesh.vertices,
                                self.objtrimesh.face_normals,
                                self.objtrimesh.faces)
        geomnodeobj = GeomNode('obj')
        geomnodeobj.addGeom(geom)
        self.npnodeobj = NodePath('obj')
        self.npnodeobj.attachNewNode(geomnodeobj)
        self.npnodeobj.setColor(0,0,0.5,0.2)
        self.npnodeobj.setTransparency(TransparencyAttrib.M_dual)
        

    def loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:
        """
        
        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, handname = self.handname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def randomlyPickOneGrasp(self):
        random_index = random.randint(0, len(self.freegripid))
        return [self.freegriprotmats[random_index], self.freegripjawwidth[random_index]]

    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])

    def getPrePickupPose(self, grasppose, jawwidth):

        # open the hand to ensure it doesnt collide with surrounding obstacles
        self.hand.setJawwidth(jawwidth)
        self.hand.setMat(pandanpmat4 = grasppose)

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(grasppose, cct0)
        cct1 = self.getPointFromPose(grasppose, cct1)

        toGroundDirection = np.array([self.objcom[0] - (cct0[0] + cct1[0])/2, 
                                      self.objcom[1] - (cct0[1] + cct1[1])/2, 
                                      self.objcom[2] - (cct0[2] + cct1[2])/2])

        liftuppose = trigeom.align_vectors(toGroundDirection, [0,0,-1])

        self.npnodeobj.setMat(pg.cvtMat4np4(liftuppose))

        liftuppose[2,3] = -self.npnodeobj.getTightBounds()[0][2]

        return pg.cvtMat4np4(liftuppose)

    def showPickUp(self, base, placementpose, grasppose, jawwidth):

        self.hand.setJawwidth(jawwidth)
        self.hand.setMat(pandanpmat4 = grasppose * placementpose)

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(grasppose, cct0)
        cct1 = self.getPointFromPose(grasppose, cct1)

        # show mass center
        pandageom.plotSphere(base.render, pos=self.getPointFromPose(placementpose, Point3(self.objcom[0], self.objcom[1] , self.objcom[2])), radius=5, rgba=Vec4(1,0,0,1))

        pandageom.plotSphere(base.render, pos=self.getPointFromPose(placementpose, Point3((cct0[0] + cct1[0])/2, (cct0[1] + cct1[1])/2, (cct0[2] + cct1[2])/2)), radius=5, rgba=Vec4(1,1,0,1))

        pandageom.plotArrow(base.render, spos=self.getPointFromPose(placementpose, Point3((cct0[0] + cct1[0])/2, (cct0[1] + cct1[1])/2, (cct0[2] + cct1[2])/2)),
                                         epos=self.getPointFromPose(placementpose, Point3(self.objcom[0], self.objcom[1] , self.objcom[2])),
                                         length=100,
                                         rgba=Vec4(0,0,1,1))

        self.hand.reparentTo(base.render)

        # plot the object
        self.npnodeobj.setMat(placementpose)
        self.npnodeobj.reparentTo(base.render)

        # self.npnodeobj.showTightBounds()

if __name__ == '__main__':
    base = pandactrl.World(camp=[700,700,700], lookatp=[0,0,0], focusLength=1212)
    this_dir, this_filename = os.path.split(__file__)

    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    objpath = os.path.join(this_dir, "objects", "cup.stl")
    # objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "good_book.stl")
    # objpath = os.path.join(this_dir, "objects", "cylinder.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")
    # objpath = os.path.join(this_dir, "objects", "Lshape.stl")

    handpkg = fetch_grippernm
    gdb = db.GraspDB()

    pickup_planner = StablePickupPlanner(objpath, handpkg, gdb)

    input_grasp = pickup_planner.randomlyPickOneGrasp()

    liftuppose = pickup_planner.getPrePickupPose(input_grasp[0], input_grasp[1])

    pickup_planner.showPickUp(base, liftuppose, input_grasp[0], input_grasp[1])

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont

    bullcldrnp = base.render.attachNewNode("bulletcollider")
    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(True)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    debugNP.show()
    
    pickup_planner.bulletworldhp.setDebugNode(debugNP.node())
    
    taskMgr.add(updateworld, "updateworld", extraArgs=[pickup_planner.bulletworldhp], appendTask=True)


    base.run()