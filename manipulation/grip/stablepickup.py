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
        self.planebullnode = cd.genCollisionPlane(offset=0, name="ground_collision")
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
        geomnodeobj = GeomNode('obj_geom')
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
        # random_index = 19
        print("random index ", random_index)
        return [self.freegriprotmats[random_index], self.freegripjawwidth[random_index]]

    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])


    def checkWayToPlace(self, base, liftupPose, grasppose, jawwidth):
        '''
        According to the paper "Reorienting Objects in 3D Space Using Pivoting", 
        non-slipping placing down requires that both ground-object contact point and masscenter
        is on one of the side of the hand-object contact point.

        We try to find such placement without slippage.

        We assume there is only one contact point.

        There are two cases. First, all ground contact point, mass center, and hand contact point are on the 
        same line. The second case is that they are not on the same line.
        '''
        def angle_between(p1, p2):
            ang1 = np.arctan2(*p1[::-1])
            ang2 = np.arctan2(*p2[::-1])
            return np.rad2deg((ang1 - ang2) % (2 * np.pi))

        self.hand.setJawwidth(jawwidth)
        self.hand.setMat(pandanpmat4 = grasppose * liftupPose)

        rotatePlaneNormal = self.getPointFromPose(grasppose * liftupPose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose * liftupPose, Point3(0, 0, 0))

        z_axis_rotation = pandageom.cvtMat4(rm.rodrigues([0,0,1], angle_between([rotatePlaneNormal[0], rotatePlaneNormal[1]], [0, 1])))

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(grasppose, cct0)
        cct1 = self.getPointFromPose(grasppose, cct1)

        mass2d_x = self.getPointFromPose(liftupPose*z_axis_rotation, Point3(self.objcom[0], self.objcom[1], self.objcom[2]))[0]

        # find the closest ground contact point
        # move the object down a little to cause collision
        liftuppose.setCell(3,2,liftuppose.getCell(3,2) - 1)
        self.npnodeobj.setMat(liftupPose)

        objbullnode = cd.genCollisionMeshMultiNp(self.npnodeobj, name="obj_collision")
        result = self.bulletworldhp.contactTest(objbullnode)

        ground_touching_point_2d = []
        ground_touching_point_3d = []

        for c in range(result.getNumContacts()):
            contact = result.getContact(c)
            contactpointOnObject = contact.getManifoldPoint().get_position_world_on_b()
            contactpointOnObject[2] = contactpointOnObject[2] + 1
            ground_touching_point_2d.append(self.getPointFromPose(z_axis_rotation, contactpointOnObject)[0])
            ground_touching_point_3d.append(contactpointOnObject)

        liftuppose.setCell(3,2,liftuppose.getCell(3,2) + 1)
        self.npnodeobj.setMat(liftupPose)

        # the closest x value of ground contact point
        cloestIndex = np.argmin(map(lambda x:abs(x - mass2d_x), ground_touching_point_2d))

        # we should have the closest ground contact point to the mass center
        cloestValue = ground_touching_point_2d[cloestIndex]
        rotationPoint = ground_touching_point_3d[cloestIndex]

        # find the lowest point on convex on each side
        left_rotation_angle_2d, right_rotation_angle_2d = 1.57, 1.57
        left_contact_point, right_contact_point = None, None

        for i in range(len(self.objtrimeshconv.faces)):
            for j in range(3):
                vert = self.objtrimeshconv.vertices[self.objtrimeshconv.faces[i][j]]
                vertp = self.getPointFromPose(liftuppose * z_axis_rotation, vert)

                # if the other side of touch point is so close to the rotate point, then ignore it
                if np.linalg.norm([abs(cloestValue - vertp[0]), vertp[2]]) < 1.0:
                    continue

                if vertp[0] > cloestValue:
                    temp = np.arctan2(vertp[2], vertp[0] - cloestValue)
                    if temp < left_rotation_angle_2d and temp >= 0.0:
                        left_rotation_angle_2d = temp
                        left_contact_point = self.getPointFromPose(liftuppose, vert)

                elif vertp[0] < cloestValue:
                    temp = np.arctan2(vertp[2], cloestValue - vertp[0])
                    if temp < right_rotation_angle_2d and temp >= 0.0:
                        right_rotation_angle_2d = temp
                        right_contact_point = self.getPointFromPose(liftuppose, vert)
                else:
                    continue

        pandageom.plotSphere(base.render, pos=left_contact_point, radius=5, rgba=Vec4(0.5,0.8,0,1))
        pandageom.plotSphere(base.render, pos=right_contact_point, radius=5, rgba=Vec4(0.5,0.8,0,1))

        placedownpose = LMatrix4f(liftuppose)
        placedownpose.setCell(3,0,placedownpose.getCell(3,0) - rotationPoint[0])
        placedownpose.setCell(3,1,placedownpose.getCell(3,1) - rotationPoint[1])
        placedownpose.setCell(3,2,placedownpose.getCell(3,2) - rotationPoint[2])

        print("cloest value ", cloestValue , " mass2d x ", mass2d_x)
        print("left rotate angle ", left_rotation_angle_2d, " right rotate angle ", right_rotation_angle_2d)

        if cloestValue < mass2d_x:
            # rotate to counterclockwise
            print("rotate to counterclockwise")
            print("with angle ", left_rotation_angle_2d * 180.0 / np.pi)
            print("left contact point ", left_contact_point)
            rotmat = rm.rodrigues([rotatePlaneNormal[0], rotatePlaneNormal[1], rotatePlaneNormal[2]], left_rotation_angle_2d * 180.0 / np.pi)
            placedownpose = placedownpose * pandageom.cvtMat4(rotmat)

        elif cloestValue > mass2d_x:
            #rotate to clockwise
            print("rotate to clockwise")
            print("with angle ", right_rotation_angle_2d * 180.0 / np.pi)
            print("right contact point ", right_contact_point)
            rotmat = rm.rodrigues([rotatePlaneNormal[0], rotatePlaneNormal[1], rotatePlaneNormal[2]], -right_rotation_angle_2d * 180.0 / np.pi)
            placedownpose = placedownpose * pandageom.cvtMat4(rotmat)

        placedownpose.setCell(3,0,placedownpose.getCell(3,0) + rotationPoint[0])
        placedownpose.setCell(3,1,placedownpose.getCell(3,1) + rotationPoint[1])
        placedownpose.setCell(3,2,placedownpose.getCell(3,2) + rotationPoint[2])

        return placedownpose

        



    def getPrePickupPose(self, grasppose, jawwidth):
        '''
        Given an initial grasp, this function return the pre-lift up pose the object should be.
        '''

        # open the hand to ensure it doesnt collide with surrounding obstacles
        self.hand.setJawwidth(jawwidth)
        self.hand.setMat(pandanpmat4 = grasppose)

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(grasppose, cct0)
        cct1 = self.getPointFromPose(grasppose, cct1)

        toGroundDirection = np.array([self.objcom[0] - (cct0[0] + cct1[0])/2, 
                                      self.objcom[1] - (cct0[1] + cct1[1])/2, 
                                      self.objcom[2] - (cct0[2] + cct1[2])/2])

        liftupPose = trigeom.align_vectors(toGroundDirection, [0,0,-1])

        self.npnodeobj.setMat(pg.cvtMat4np4(liftupPose))

        MinPt, MaxPt = Point3(), Point3()

        self.npnodeobj.calc_tight_bounds(MinPt, MaxPt)

        liftupPose[2,3] = -MinPt[2]

        return pg.cvtMat4np4(liftupPose)

    def showPickUp(self, base, placementpose, grasppose, jawwidth):

        tmphand = handpkg.newHandNM(hndcolor=[0,1,0,.7])

        tmphand.setJawwidth(jawwidth)
        tmphand.setMat(pandanpmat4 = grasppose * placementpose)

        pandageom.plotArrow(base.render, spos=self.getPointFromPose(grasppose * placementpose, Point3(0, 0, 0)),
                                         epos=self.getPointFromPose(grasppose * placementpose, Point3(0, 1, 0)),
                                         length=40,
                                         rgba=Vec4(0,1,0,1))

        cct0, cct1 = tmphand.getFingerTips()
        cct0 = self.getPointFromPose(grasppose, cct0)
        cct1 = self.getPointFromPose(grasppose, cct1)

        # show mass center
        pandageom.plotSphere(base.render, pos=self.getPointFromPose(placementpose, Point3(self.objcom[0], self.objcom[1] , self.objcom[2])), radius=5, rgba=Vec4(1,0,0,1))

        pandageom.plotSphere(base.render, pos=self.getPointFromPose(placementpose, Point3((cct0[0] + cct1[0])/2, (cct0[1] + cct1[1])/2, (cct0[2] + cct1[2])/2)), radius=5, rgba=Vec4(1,1,0,1))

        pandageom.plotArrow(base.render, spos=self.getPointFromPose(placementpose, Point3((cct0[0] + cct1[0])/2, (cct0[1] + cct1[1])/2, (cct0[2] + cct1[2])/2)),
                                         epos=self.getPointFromPose(placementpose, Point3(self.objcom[0], self.objcom[1] , self.objcom[2])),
                                         length=100,
                                         rgba=Vec4(0,0,1,1))

        tmphand.reparentTo(base.render)

        geom = pg.packpandageom(self.objtrimesh.vertices,
                                self.objtrimesh.face_normals,
                                self.objtrimesh.faces)

        geomnodeobj = GeomNode('obj_geom')
        geomnodeobj.addGeom(geom)
        objecttemp = NodePath('obj')
        objecttemp.attachNewNode(geomnodeobj)
        objecttemp.setColor(0,0,0.5,0.2)
        objecttemp.setTransparency(TransparencyAttrib.M_dual)

        # plot the object
        objecttemp.setMat(placementpose)
        objecttemp.reparentTo(base.render)

        # self.npnodeobj.showTightBounds()

if __name__ == '__main__':
    base = pandactrl.World(camp=[700,700,700], lookatp=[0,0,0], focusLength=1212)
    this_dir, this_filename = os.path.split(__file__)

    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    objpath = os.path.join(this_dir, "objects", "book.stl")
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
    placedownPose = pickup_planner.checkWayToPlace(base, liftuppose, input_grasp[0], input_grasp[1])
    pickup_planner.showPickUp(base, liftuppose, input_grasp[0], input_grasp[1])
    pickup_planner.showPickUp(base, placedownPose, input_grasp[0], input_grasp[1])

    


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

    axis = loader.loadModel('zup-axis.egg')
    axis.setScale(10)
    axis.reparentTo(base.render)


    base.run()