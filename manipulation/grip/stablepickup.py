#!/user/bin/python
import sys
import os
import numpy as np

from direct.showbase import DirectObject

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

from ffregrasp import ff_regrasp_planner

import random

import networkx as nx
import math
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R

from direct.task import Task
from time import sleep

# fibonacci sphere points generator
def fibonacci_sphere(samples=100):

    points = []
    phi = math.pi * (3.0 - math.sqrt(5.0))  # golden angle in radians

    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2  # y goes from 1 to -1
        radius = math.sqrt(1 - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        points.append((x, y, z))

    return points

def mapParallelDirection(d):
    if d[0] < 0.0 or (d[0] == 0.0 and d[1] < 0.0) or (d[0] == 0.0 and d[1] == 0.0 and d[2] < 0.0):
        return (-d[0], -d[1], -d[2])
    return (d[0], d[1], d[2])

class StablePickupPlanner(object):
    def __init__(self, objpath, handpkg, gdb):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.objcom = self.objtrimesh.center_mass
        self.objcom[abs(self.objcom) < 1e-10] = 0
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
        self.validangle = 0.2

        self.gdb = gdb
        self.loadFreeAirGrip()
        self.loadFreeTablePlacement()

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

    def loadFreeTablePlacement(self):
        """
        This function will return a list of placement poses, placement ids, and placement types.
        The first part of the list are stable placements, while the second part of the list are unstable placements.
        """
        freetabletopplacementdata = self.gdb.loadFreeTabletopPlacementIncludeFF(self.dbobjname)
        if freetabletopplacementdata is None:
            raise ValueError("Plan the freeairgrip first!")
        self.tpsmat4s, self.placementid, self.placementtype = freetabletopplacementdata

    def randomlyPickOneGrasp(self):
        """
        generate a random initial grasp and placement in numpy format
         
        """
        random_index = random.randint(0, len(self.freegripid))
        # random_index = 0
        # random_index = 456
        print("random index ", random_index)

        # get random placement where the grasp is valid
        sql = "SELECT freetabletopplacement.rotmat FROM freetabletopgrip, freetabletopplacement, object WHERE freetabletopplacement.idobject = object.idobject \
                AND object.name LIKE '%s' AND freetabletopgrip.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement \
                AND freetabletopgrip.idfreeairgrip = %d" % (self.dbobjname, self.freegripid[random_index])
        result = self.gdb.execute(sql)
        if len(result) == 0:
            print("there is no way to place the object with current grasp")
            return None, None
        random_placement_index = random.randint(0, len(result) - 1)
        # random_placement_index = 1
        # random_placement_index = 0
        print("check random placement id", random_placement_index)

        return [pg.mat4ToNp(self.freegriprotmats[random_index]), self.freegripjawwidth[random_index]], pg.mat4ToNp(dc.strToMat4(result[random_placement_index][0]))
        
    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])

    def checkWayToPlace(self, liftupPose_t, grasppose_t, jawwidth):
        '''
        According to the paper "Reorienting Objects in 3D Space Using Pivoting", 
        non-slipping placing down requires that both ground-object contact point and masscenter
        is on one of the side of the hand-object contact point.

        We try to find such placement without slippage.

        We assume there is only one contact point.

        There are two cases. First, all ground contact point, mass center, and hand contact point are on the 
        same line. The second case is that they are not on the same line.
        '''
        liftupPose = pg.cvtMat4np4(liftupPose_t)
        grasppose = pg.cvtMat4np4(grasppose_t)

        def getAngleWithRotationMatrix(rotatematrix_, direction_):
            temp = rm.transformmat4(rotatematrix_, direction_)
            return np.arctan2(temp[1], temp[0])

        def findWhichRangeDirectionBelongto(anglelist, direction):
            maxindex = np.argmax(anglelist)
            minindex = np.argmin(anglelist)
            if direction > anglelist[maxindex] or direction < anglelist[minindex]:
                left = min(minindex, maxindex)
                right = max(minindex, maxindex)
                if left == 0 and right == len(anglelist) - 1:
                    return [right, left]
                else:
                    return [left, right]

            for f in range(len(anglelist)):
                left = f
                right = f + 1 if f < len(anglelist) - 1 else 0
                if (left == maxindex and right == minindex) or (left == minindex and right == maxindex):
                    continue
                if (anglelist[left] > direction and anglelist[right] < direction) or (anglelist[left] < direction and anglelist[right] > direction):
                    if left == 0 and right == len(anglelist) - 1:
                        return [right, left]
                    else:
                        return [left, right]

        # in this section , we first find both ff or stable placement which are on the both side of current lifted up pose
        # then find the all corner points between this two placements, so we can find how to rotate to both way

        currentfignerdirection = np.array(self.getPointFromPose(grasppose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose, Point3(0, 0, 0)))

        possibleplacements, possibleplacementdirections, rotatecorners, tempplacements = pg.generateFFPlacement(self.objtrimeshconv, 
                                                currentfignerdirection, 
                                                self.objcom, 0.9)

        rotatematrix = trigeom.align_vectors(currentfignerdirection, [0,0,1])

        currentplacementdirection2d = getAngleWithRotationMatrix(rotatematrix, pg.getGroundDirection(liftupPose_t))
        placementangles = [getAngleWithRotationMatrix(rotatematrix, possibleplacementdirections[p]) for p in range(len(possibleplacementdirections))]
        twonearplacementdirectionindex = findWhichRangeDirectionBelongto(placementangles, currentplacementdirection2d)

        pivotingplacements = [possibleplacements[twonearplacementdirectionindex[0]]] + tempplacements[twonearplacementdirectionindex[0]] + [possibleplacements[twonearplacementdirectionindex[1]]]

        # find where current object pose is in.
        placementdirectionangles = []

        placementdirectionangles.append(getAngleWithRotationMatrix(rotatematrix, possibleplacementdirections[twonearplacementdirectionindex[0]]))
        for cp in tempplacements[twonearplacementdirectionindex[0]]:
            placementdirectionangles.append(getAngleWithRotationMatrix(rotatematrix, pg.getGroundDirection(cp)))
        placementdirectionangles.append(getAngleWithRotationMatrix(rotatematrix, possibleplacementdirections[twonearplacementdirectionindex[1]]))

        twoplacements = findWhichRangeDirectionBelongto(placementdirectionangles, currentplacementdirection2d)

        # need to check collision of the gripper so we can determine which way should be used.

        leftwaycorner = rotatecorners[twonearplacementdirectionindex[0]][:twoplacements[1]]
        leftwayplacements = pivotingplacements[:twoplacements[1]]

        leftAvalibility = True
        rightAvalibility = True

        for leftposes in leftwayplacements:
            self.hand.setJawwidth(jawwidth)
            self.hand.setMat(pandanpmat4 = grasppose * pg.cvtMat4np4(leftposes))
            if self.bulletworldhp.contactTest(cd.genCollisionMeshMultiNp(self.hand.handnp)).getNumContacts():
                leftAvalibility = False
                break


        rightwaycorner = list(reversed(rotatecorners[twonearplacementdirectionindex[0]][twoplacements[0]:]))
        rightwayplacements = list(reversed(pivotingplacements[twoplacements[1]:]))

        for rightposes in rightwayplacements:
            self.hand.setJawwidth(jawwidth)
            self.hand.setMat(pandanpmat4 = grasppose * pg.cvtMat4np4(rightposes))
            if self.bulletworldhp.contactTest(cd.genCollisionMeshMultiNp(self.hand.handnp)).getNumContacts():
                rightAvalibility = False
                break

        def angleDiff(target, source):
            a = target - source
            if a > np.pi:
                a -= (2.0 * np.pi)
            if a < -np.pi:
                a += (2.0 * np.pi)
            return abs(a)

        if not leftAvalibility and rightAvalibility:
            return rightwayplacements[0], rightwayplacements, rightwaycorner, self.getPointFromPose(grasppose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose, Point3(0, 0, 0))
        elif not rightAvalibility and leftAvalibility:
            return leftwayplacements[0], leftwayplacements, leftwaycorner, self.getPointFromPose(grasppose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose, Point3(0, 0, 0))
        elif angleDiff(placementdirectionangles[0], currentplacementdirection2d) < angleDiff(placementdirectionangles[1], currentplacementdirection2d):
            return leftwayplacements[0], leftwayplacements, leftwaycorner, self.getPointFromPose(grasppose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose, Point3(0, 0, 0))
        else:
            return rightwayplacements[0], rightwayplacements, rightwaycorner, self.getPointFromPose(grasppose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose, Point3(0, 0, 0))
        



    def getPrePickupPose(self, grasppose, jawwidth):
        '''
        Given an initial grasp in numpy format, this function return the pre-lift up pose the object should be.

        '''

        # open the hand to ensure it doesnt collide with surrounding obstacles
        self.hand.setJawwidth(jawwidth)
        self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(grasppose))

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(pg.cvtMat4np4(grasppose), cct0)
        cct1 = self.getPointFromPose(pg.cvtMat4np4(grasppose), cct1)

        toGroundDirection = np.array([self.objcom[0] - (cct0[0] + cct1[0])/2, 
                                      self.objcom[1] - (cct0[1] + cct1[1])/2, 
                                      self.objcom[2] - (cct0[2] + cct1[2])/2])

        liftupPose = trigeom.align_vectors(toGroundDirection, [0,0,-1])

        self.npnodeobj.setMat(pg.cvtMat4np4(liftupPose))

        MinPt, MaxPt = Point3(), Point3()

        self.npnodeobj.calc_tight_bounds(MinPt, MaxPt)

        liftupPose[2,3] = -MinPt[2]

        return liftupPose
        # return pg.cvtMat4np4(liftupPose)

    def showPickUp(self, base, placementpose, grasppose, jawwidth):
        """
        Given placement pose and grasp pose in numpy format and visualize them.
        """

        tmphand = handpkg.newHandNM(hndcolor=[0,1,0,.7])

        tmphand.setJawwidth(jawwidth)
        tmphand.setMat(pandanpmat4 = pg.cvtMat4np4(grasppose) * pg.cvtMat4np4(placementpose))

        pandageom.plotArrow(base.render, spos=self.getPointFromPose(pg.cvtMat4np4(grasppose) * pg.cvtMat4np4(placementpose), Point3(0, 0, 0)),
                                         epos=self.getPointFromPose(pg.cvtMat4np4(grasppose) * pg.cvtMat4np4(placementpose), Point3(0, 1, 0)),
                                         length=40,
                                         rgba=Vec4(0,1,0,1))

        cct0, cct1 = tmphand.getFingerTips()
        cct0 = self.getPointFromPose(pg.cvtMat4np4(grasppose), cct0)
        cct1 = self.getPointFromPose(pg.cvtMat4np4(grasppose), cct1)

        # show mass center
        pandageom.plotSphere(base.render, pos=self.getPointFromPose(pg.cvtMat4np4(placementpose), Point3(self.objcom[0], self.objcom[1] , self.objcom[2])), radius=5, rgba=Vec4(1,0,0,1))

        pandageom.plotSphere(base.render, pos=self.getPointFromPose(pg.cvtMat4np4(placementpose), Point3((cct0[0] + cct1[0])/2, (cct0[1] + cct1[1])/2, (cct0[2] + cct1[2])/2)), radius=5, rgba=Vec4(1,1,0,1))

        pandageom.plotArrow(base.render, spos=self.getPointFromPose(pg.cvtMat4np4(placementpose), Point3((cct0[0] + cct1[0])/2, (cct0[1] + cct1[1])/2, (cct0[2] + cct1[2])/2)),
                                         epos=self.getPointFromPose(pg.cvtMat4np4(placementpose), Point3(self.objcom[0], self.objcom[1] , self.objcom[2])),
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
        objecttemp.setMat(pg.cvtMat4np4(placementpose))
        # objecttemp.setMat(pg.cvtMat4np4(np.identity(4)))
        objecttemp.reparentTo(base.render)

    
    def getPlaneWithTwoPoints(self, p1, p0):
        """
        Given two points on the different size of a plane, this function will return the plane's value (position, normal direction).
        """
        normal_direction = np.array([p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]])
        normal_direction = normal_direction / np.linalg.norm(normal_direction)
        plane_center = np.array([p1[0] + p0[0], p1[1] + p0[1], p1[2] + p0[2]]) / 2
        return np.concatenate((plane_center, normal_direction))

    def generateManipulationPlane(self):
        '''
        According to the object mesh, it will generate a set of manipulation plane with contact points on it
        '''

        # use a fibonacci method to generate a set of direction in 2d
        possibleFingerDirections = fibonacci_sphere(642)

        # each i-th list of graspsInDirection contains a set of grasps belong the plane whose normal is equal to i-th accuracyDirect.
        graspsInDirection = [[] for _ in range(len(possibleFingerDirections))] # possible finger directions
        accuracyDirect = np.zeros((len(possibleFingerDirections), 3))

        # build the kd tree of the direction sets
        tpsdirections = KDTree(np.array(possibleFingerDirections))

        # cluster grasps according to the plane
        for f in range(len(self.freegripnormals)):
            _, ind = tpsdirections.query(mapParallelDirection(self.freegripnormals[f][0]))
            if len(graspsInDirection[ind]) == 0:
                newFreegripnormals = mapParallelDirection(self.freegripnormals[f][0])
                accuracyDirect[ind][0] = newFreegripnormals[0]
                accuracyDirect[ind][1] = newFreegripnormals[1]
                accuracyDirect[ind][2] = newFreegripnormals[2]
            graspsInDirection[ind].append([self.freegriprotmats[f], 
                                          self.freegripjawwidth[f], 
                                          self.freegripid[f], 
                                          self.freegripnormals[f][0], 
                                          self.freegripnormals[f][1]])

        # fingeridrections keeps the pair(normal direction of a set of grasp, direction id)
        fingerdirections = []
        contactpointInPlane = []
        graspsIdInPlane = []
        for n in range(len(graspsInDirection)):
            # for each finger direction
            if len(graspsInDirection[n]) > 0:
                # if there is some grasps in this direction
                fingerdirections.append(accuracyDirect[n])
                contactpoints = []
                graspids = []
                for i in range(len(graspsInDirection[n])):
                    # find the contact point for each grasp
                    contactpoints.append(rm.transformmat4(np.transpose(graspsInDirection[n][i][0]), [self.hand.contactPointOffset, 0, 0]))
                    graspids.append(graspsInDirection[n][i][2])
                contactpointInPlane.append(contactpoints)
                graspsIdInPlane.append(graspids)

        return fingerdirections, contactpointInPlane, graspsIdInPlane

    def getdmgid(self, placementid_, fingerdirection):
        sql = "SELECT dmgs.iddmg, dmgs.planevector FROM dmgs WHERE dmgs.placementid=%d " % placementid_
        for dmgid, dmgplane in self.gdb.execute(sql):
            dmg_plane = dc.strToV6(dmgplane)
            if np.linalg.norm(fingerdirection - dmg_plane[3:6]) <= 0.1:
                return dmgid
        return None    

    def generateManipulationCircle(self, fingerdirections, contactpointInPlane, graspsIdInPlane):

        placement2placement = []
        # according to all possible finger directions, a set of placements can be pivoted by that grasp
        # the placements is in a loop according to the fingertip direction
        for fingerdirection, currentContactPoints, currentGraspsIds in zip(fingerdirections, contactpointInPlane, graspsIdInPlane):
            # get all grasps belong to this finger direction
            graspsBelongToCurrentPlane = np.stack(currentContactPoints)[:, :3]

            # ffdirections is the placement direction in the object frame
            # rotate corners are the rotate corners in the object frame
            _, ffdirections, rotateCornersInfo, rotateplacements = pg.generateFFPlacement(self.objtrimeshconv, fingerdirection, self.objcom, 0.9)
            ffdirectionsMat = np.stack(ffdirections)

            manipulationPlaneXAxisMat = (np.array([
                    [0, -fingerdirection[2], fingerdirection[1]],
                    [fingerdirection[2], 0, -fingerdirection[0]],
                    [-fingerdirection[1], fingerdirection[0], 0]
                    ]).dot(ffdirectionsMat.transpose())).transpose()

            manipulation_planeMat = np.stack([manipulationPlaneXAxisMat, ffdirectionsMat])

            massCenterInPlaneMat = (np.tensordot(self.objcom, manipulation_planeMat, (0,2)) / np.sum(manipulation_planeMat*manipulation_planeMat, axis=2)).transpose()

            rotateCorners = []
            validCornerBit = []

            for l in range(len(rotateCornersInfo)):
                rotateCorners.append(np.array([x for x, _ in rotateCornersInfo[l]]))
                validCornerBit.append(np.array([x for _, x in rotateCornersInfo[l]]))

            for l in range(len(rotateCorners)):
                if not validCornerBit[l].all(): # if some corners are not stable then skip
                    continue
                rotateCornersInPlane = rotateCorners[l].dot(manipulation_planeMat[:, l, :].transpose()) / np.sum(manipulation_planeMat[:, l, :]*manipulation_planeMat[:, l, :], axis=1)
                graspsInCurrentPlane = graspsBelongToCurrentPlane.dot(manipulation_planeMat[:, l, :].transpose() / np.sum(manipulation_planeMat[:, l, :]*manipulation_planeMat[:, l, :], axis=1))
                massCenterDirectionMat = (massCenterInPlaneMat[l] - rotateCornersInPlane)

                commonvalidgrasps = np.array([], dtype=np.bool).reshape(0, graspsInCurrentPlane.shape[0])

                for c in range(rotateCornersInPlane.shape[0]):
                    # for each corners, calculate the direction from corner to grasps contact points
                    graspsdirections = graspsInCurrentPlane - rotateCornersInPlane[c] # shape (# of grasps, 2)
                    # check whether the grasps length is longer than the mass center length
                    commonvalidgrasps = np.vstack((commonvalidgrasps, (np.linalg.norm(graspsdirections, axis=1)[:, np.newaxis].transpose() > np.linalg.norm(massCenterDirectionMat[c]))))
                    # check if the angle between current grasp direction and mass angle is too large
                    graspsdirections = graspsdirections / np.linalg.norm(graspsdirections, axis=1)[:, np.newaxis]
                    massdirectiontemp = massCenterDirectionMat[c] / np.linalg.norm(massCenterDirectionMat[c])
                    commonvalidgrasps = np.vstack((commonvalidgrasps, np.arccos(graspsdirections.dot(massdirectiontemp)) < self.validangle))

                valid_common_grasp_bit = commonvalidgrasps.all(0)

                if valid_common_grasp_bit.any():
                    # calculate the placement pose with placement direction
                    placementpose1 = self.placementdirection2pose(ffdirections[l])
                    placementpose2 = self.placementdirection2pose(ffdirections[(l + 1) % len(rotateCorners)])

                    placement1poseid, placement1type = self.getPlacementIdFromPose(placementpose1)
                    placement2poseid, placement2type = self.getPlacementIdFromPose(placementpose2)

                    # need to verify the grasp has no collision with the ground in two different placements
                    positiveValidPivotGraspid = []
                    negativeValidPivotGraspid = []

                    for graspid in np.array(currentGraspsIds)[valid_common_grasp_bit]:

                        pivotgrasp, pivotgraspJawwidth = self.gdb.loadFreeAirGripByIds(graspid)
                        self.hand.setJawwidth(pivotgraspJawwidth)
                        self.hand.setMat(pandanpmat4 = pivotgrasp * pg.cvtMat4np4(placementpose1))
                        # add hand model to bulletworld
                        hndbullnode = cd.genCollisionMeshMultiNp(self.hand.handnp)
                        result1 = self.bulletworldhp.contactTest(hndbullnode)

                        self.hand.setMat(pandanpmat4 = pivotgrasp * pg.cvtMat4np4(placementpose2))
                        # add hand model to bulletworld
                        hndbullnode = cd.genCollisionMeshMultiNp(self.hand.handnp)
                        result2 = self.bulletworldhp.contactTest(hndbullnode)


                        if not result1.getNumContacts() and not result2.getNumContacts():
                            # find grasp direction
                            cct0, cct1 = self.hand.getFingerTips()
                            cct0 = self.getPointFromPose(pivotgrasp, cct0)
                            cct1 = self.getPointFromPose(pivotgrasp, cct1)
                            if np.dot((cct1 - cct0)/np.linalg.norm(cct1-cct0), fingerdirection) > 0:
                                positiveValidPivotGraspid.append(graspid)
                            else:
                                negativeValidPivotGraspid.append(graspid)

                    if placement1type == 0 and placement2type == 0:
                        placement2placement.append([placement1poseid, placement2poseid , placement1poseid, placement2poseid, positiveValidPivotGraspid + negativeValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])
                    elif placement1type == 0 and placement2type == 1:
                        cct0 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose2), [0,0,0]))
                        cct1 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose2), fingerdirection))
                        positiveplacement2nodeid = self.getdmgid(placement2poseid, (cct1-cct0)/np.linalg.norm(cct1-cct0))
                        negativeplacement2nodeid = self.getdmgid(placement2poseid, (cct0-cct1)/np.linalg.norm(cct0-cct1))
                        placement2placement.append([placement1poseid, positiveplacement2nodeid , placement1poseid, placement2poseid, positiveValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])
                        placement2placement.append([placement1poseid, negativeplacement2nodeid , placement1poseid, placement2poseid, negativeValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])
                    elif placement1type == 1 and placement2type == 0:
                        cct0 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose1), [0,0,0]))
                        cct1 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose1), fingerdirection))
                        positiveplacement1nodeid = self.getdmgid(placement1poseid, (cct1-cct0)/np.linalg.norm(cct1-cct0))
                        negativeplacement1nodeid = self.getdmgid(placement1poseid, (cct0-cct1)/np.linalg.norm(cct0-cct1))
                        placement2placement.append([positiveplacement1nodeid, placement2poseid , placement1poseid, placement2poseid, positiveValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])
                        placement2placement.append([negativeplacement1nodeid, placement2poseid , placement1poseid, placement2poseid, negativeValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])
                    elif placement1type == 1 and placement2type == 1:
                        cct0 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose1), [0,0,0]))
                        cct1 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose1), fingerdirection))
                        positiveplacement1nodeid = self.getdmgid(placement1poseid, (cct1-cct0)/np.linalg.norm(cct1-cct0))
                        negativeplacement1nodeid = self.getdmgid(placement1poseid, (cct0-cct1)/np.linalg.norm(cct0-cct1))

                        cct0 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose2), [0,0,0]))
                        cct1 = np.array(self.getPointFromPose(pg.cvtMat4np4(placementpose2), fingerdirection))
                        positiveplacement2nodeid = self.getdmgid(placement2poseid, (cct1-cct0)/np.linalg.norm(cct1-cct0))
                        negativeplacement2nodeid = self.getdmgid(placement2poseid, (cct0-cct1)/np.linalg.norm(cct0-cct1))
                        placement2placement.append([positiveplacement1nodeid, positiveplacement2nodeid , placement1poseid, placement2poseid, positiveValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])
                        placement2placement.append([negativeplacement1nodeid, negativeplacement2nodeid , placement1poseid, placement2poseid, negativeValidPivotGraspid, (rotateCorners[l], fingerdirection, rotateplacements[l])])


        return placement2placement

    def placementdirection2pose(self, direction):
        # this function will convert the placement direction according to the placement pose

        pose = trigeom.align_vectors(direction, [0,0,-1])

        self.npnodeobj.setMat(pg.cvtMat4np4(pose))

        MinPt, MaxPt = Point3(), Point3()

        self.npnodeobj.calc_tight_bounds(MinPt, MaxPt)

        pose[2,3] = -MinPt[2]

        return pose

        # return pg.cvtMat4np4(pose)

    def getPivotTrajectory(self, placementpose1, placementpose2, pivotPoint):
        """
        Given the beginning placement and target placement, we use the pivotPoint to find the final placement after pivoting.
        placementpose1: first placement pose
        placementpose2: second placement pose
        the pivot point list in the object frame
        where pivotPoint[0] is a list of pivot poing, while pivotPoint[1] is the rotation axis
        """

        # calculate the pivoted pose
        p1 = rm.transformmat4(placementpose1, pivotPoint[0] + pivotPoint[1])[:3] - rm.transformmat4(placementpose1, pivotPoint[0])[:3]

        cornerpoint1 = np.identity(4)
        cornerpoint1[:3, :3] = np.vstack([p1, np.dot(rm.hat(p1), np.array([0,0,1.0])), np.array([0,0,1.0])]).transpose()
        cornerpoint1[:3, 3] = rm.transformmat4(placementpose1, pivotPoint[0])[:3]

        p2 = rm.transformmat4(placementpose2, pivotPoint[0] + pivotPoint[1])[:3] - rm.transformmat4(placementpose2, pivotPoint[0])[:3]

        cornerpoint2 = np.identity(4)
        cornerpoint2[:3, :3] = np.vstack([p2, np.dot(rm.hat(p2), np.array([0,0,1.0])), np.array([0,0,1.0])]).transpose()
        cornerpoint2[:3, 3] = rm.transformmat4(placementpose2, pivotPoint[0])[:3]

        pivotedPlacement = cornerpoint1.dot(np.linalg.inv(cornerpoint2)).dot(placementpose2)

        # result = [placement1pose.copy()]
        result = []

        # calculate the rotate angle
        r = R.from_dcm((np.linalg.inv(placementpose1).dot(pivotedPlacement))[:3, :3]).as_rotvec()
        rotateAngle = np.linalg.norm(r)
        rotateAxis = r / rotateAngle
                
        stepNum = 30
        stepRotation = rotateAngle / stepNum * rotateAxis

        shiftmatrix = np.identity(4)
        rotateMatrix = np.identity(4)
        shiftmatrix[:3, 3] = pivotPoint[0]
        
        for i in range(stepNum):
            rotateMatrix[:3, :3] = R.from_rotvec(stepRotation * i).as_dcm()
            result.append(placementpose1.dot(shiftmatrix).dot(rotateMatrix).dot(np.linalg.inv(shiftmatrix)))
        
        result.append(pivotedPlacement)

        return result

    def showPivot(self, pivotaction, base):
        """
        this function will visualize the pivot action
        """
        
        placementpose1, placementpose2, graspids, pivotPoint = pivotaction
        # placementpose1: first placement pose
        # placementpose2: second placement pose
        # graspids: a list of grasp ids to pivot from one placement to another
        # the pivot point in the object frame

        p1 = rm.transformmat4(placementpose1, pivotPoint[0] + pivotPoint[1])[:3] - rm.transformmat4(placementpose1, pivotPoint[0])[:3]

        cornerpoint1 = np.identity(4)
        cornerpoint1[:3, :3] = np.vstack([p1, np.dot(rm.hat(p1), np.array([0,0,1.0])), np.array([0,0,1.0])]).transpose()
        cornerpoint1[:3, 3] = rm.transformmat4(placementpose1, pivotPoint[0])[:3]

        p2 = rm.transformmat4(placementpose2, pivotPoint[0] + pivotPoint[1])[:3] - rm.transformmat4(placementpose2, pivotPoint[0])[:3]

        cornerpoint2 = np.identity(4)
        cornerpoint2[:3, :3] = np.vstack([p2, np.dot(rm.hat(p2), np.array([0,0,1.0])), np.array([0,0,1.0])]).transpose()
        cornerpoint2[:3, 3] = rm.transformmat4(placementpose2, pivotPoint[0])[:3]

        newPivotPose = cornerpoint1.dot(np.linalg.inv(cornerpoint2)).dot(placementpose2)

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
        objecttemp.setMat(pg.cvtMat4np4(placementpose1))
        objecttemp.reparentTo(base.render)

        objecttemp1 = NodePath('obj')
        objecttemp1.attachNewNode(geomnodeobj)
        objecttemp1.setColor(0.5,0,0.5,0.2)
        objecttemp1.setTransparency(TransparencyAttrib.M_dual)

        # plot the object
        objecttemp1.setMat(pg.cvtMat4np4(newPivotPose))
        objecttemp1.reparentTo(base.render)


        # show pivot point
        pandageom.plotArrow(base.render, spos=self.getPointFromPose(pg.cvtMat4np4(placementpose1), Point3(pivotPoint[0][0], pivotPoint[0][1], pivotPoint[0][2])),
                                    epos=self.getPointFromPose(pg.cvtMat4np4(placementpose1), Point3(pivotPoint[0][0] + pivotPoint[1][0], pivotPoint[0][1] + pivotPoint[1][1], pivotPoint[0][2] + pivotPoint[1][2])),
                                    length=100,
                                    rgba=Vec4(0,0,1,1))

        for graspid in graspids:

            pivotgrasp, pivotgraspJawwidth = self.gdb.loadFreeAirGripByIds(graspid)

            # draw the pivot grasp
            tmphand = handpkg.newHandNM(hndcolor=[0,1,0,.2])

            tmphand.setJawwidth(pivotgraspJawwidth)
            tmphand.setMat(pandanpmat4 = pivotgrasp * pg.cvtMat4np4(placementpose1))

            tmphand.reparentTo(base.render)

    def getPlacementIdFromPose(self, pose):
        """
        given a pose, this function will return the placement id and its placement type whose has most simlilar pose.
        """
        obj_dir_pos_2match = pg.getGroundDirection(pose)
        diff = 2.0 
        closest_placementid = None
        closest_placementtype = None
        for placementid, placementpose, placementtype in zip(self.placementid, self.tpsmat4s, self.placementtype):
            placement_pose = pg.mat4ToNp(placementpose)
            currnt_dir_pos = pg.getGroundDirection(placement_pose)
            
            currnt_diff = np.linalg.norm(currnt_dir_pos - obj_dir_pos_2match)
            if currnt_diff <= diff:
                closest_placementid = placementid
                closest_placementtype = placementtype
                diff = currnt_diff
        return closest_placementid, closest_placementtype

    def createPlacementGraph(self):
        '''
        create a pivoting graph
        '''

        # self.Graph = nx.Graph()
        self.Graph = nx.DiGraph()

        fingerdirections, contactpointInPlane, graspsIdInPlane = self.generateManipulationPlane()

        # generate a list of pair valid pivot action
        placement2placement = self.generateManipulationCircle(fingerdirections, contactpointInPlane, graspsIdInPlane)

        # build the graph
        for p in placement2placement:
            self.Graph.add_edge(p[0], p[1], placementid0=p[2], placementid1=p[3], pivotGraspids=p[4], pivotCorner=p[5])
            # flip the rotate orders
            self.Graph.add_edge(p[1], p[0], placementid0=p[3], placementid1=p[2], pivotGraspids=p[4], pivotCorner=(np.flip(p[5][0], 0), p[5][1], list(reversed(p[5][2]))))

    def getPivotPath(self, currentPlacement, currentGrasp, targetPlacement, targetGrasp):
        result = []

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(pg.cvtMat4np4(currentGrasp[0]) * pg.cvtMat4np4(currentPlacement), cct0)
        cct1 = self.getPointFromPose(pg.cvtMat4np4(currentGrasp[0]) * pg.cvtMat4np4(currentPlacement), cct1)

        currentPlacementid, currentPlacementtype = self.getPlacementIdFromPose(currentPlacement)
        currrentplacementnodeid = currentPlacementid
        if currentPlacementtype == 1:
            currrentplacementnodeid = self.getdmgid(currentPlacementid, np.array(cct1-cct0)/np.linalg.norm(cct1-cct0))

        cct0, cct1 = self.hand.getFingerTips()
        cct0 = self.getPointFromPose(pg.cvtMat4np4(targetGrasp[0]) * pg.cvtMat4np4(targetPlacement), cct0)
        cct1 = self.getPointFromPose(pg.cvtMat4np4(targetGrasp[0]) * pg.cvtMat4np4(targetPlacement), cct1)

        targetPlacementid, targetPlacementtype = self.getPlacementIdFromPose(targetPlacement)
        targetplacementnodeid = targetPlacementid
        if targetPlacementtype == 1:
            targetplacementnodeid = self.getdmgid(targetPlacementid, np.array(cct1-cct0)/np.linalg.norm(cct1-cct0))

        path = nx.shortest_path(self.Graph, currrentplacementnodeid, targetplacementnodeid)
        for i in range(len(path)-1):
            result.append([self.Graph.get_edge_data(path[i],path[i+1])['placementid0'], self.Graph.get_edge_data(path[i],path[i+1])['placementid1'], self.Graph.get_edge_data(path[i],path[i+1])['pivotGraspids'], self.Graph.get_edge_data(path[i],path[i+1])['pivotCorner']])
        return result



class DemoHelper(DirectObject.DirectObject):
    def __init__(self, objpath, handpkg, _base):
        objtrimesh=trimesh.load_mesh(objpath)
        geom = pg.packpandageom(objtrimesh.vertices,
                                objtrimesh.face_normals,
                                objtrimesh.faces)

        node = GeomNode('obj')
        node.addGeom(geom)

        self.demoObj = NodePath('obj')
        self.demoObj.attachNewNode(node)
        self.demoObj.setColor(Vec4(.7,0.3,0,0.2))
        self.demoObj.setTransparency(TransparencyAttrib.MAlpha)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.5])

        self.base = _base

        self.eventQueue = []

        self.counter = 0
        self.eventCounter = 0
        self.accept('w', self.executeEvent)

    def setObjPose(self, placementPose):
        """
        Given object pose in numpy format
        """
        self.demoObj.setMat(pg.cvtMat4np4(placementPose))
        self.demoObj.reparentTo(self.base.render)

    def setHandPose(self, handPose, jawwidth):

        self.hand.setJawwidth(jawwidth)
        self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(handPose))
        self.hand.reparentTo(self.base.render)

    def addEvent(self, action, motion):
        self.eventQueue.append([action, motion])

    def fingerGaiting(self, task, gripper_trajectory):
        if self.eventCounter > len(gripper_trajectory) - 1:
            return task.done
        self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(gripper_trajectory[self.eventCounter]))
        self.eventCounter += 1
        return task.again
    
    def pivoting(self, task, pivot_trajectory):
        if self.eventCounter > len(pivot_trajectory) - 1:
            return task.done

        # get pose of the hand
        original_object_pose = pg.mat4ToNp(self.demoObj.getMat())
        original_hand_pose = pg.mat4ToNp(self.hand.getMat())
        hand_pose_in_obj_frame = np.linalg.inv(original_object_pose).dot(original_hand_pose)

        self.demoObj.setMat(pg.cvtMat4np4(pivot_trajectory[self.eventCounter]))
        self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(pivot_trajectory[self.eventCounter].dot(hand_pose_in_obj_frame)))

        self.eventCounter += 1
        return task.again

    def executeAllEvent(self, task, eventQueue_):
        if self.counter == len(eventQueue_):
            return task.done
        
        action, trajectory = eventQueue_[self.counter]

        if self.eventCounter == len(trajectory):
            self.eventCounter = 0
            self.counter += 1
            return task.again

        if action == "fingerGait":
            self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(trajectory[self.eventCounter]))

        elif action == "pivot":
            original_object_pose = pg.mat4ToNp(self.demoObj.getMat())
            original_hand_pose = pg.mat4ToNp(self.hand.getMat())
            hand_pose_in_obj_frame = np.linalg.inv(original_object_pose).dot(original_hand_pose)

            self.demoObj.setMat(pg.cvtMat4np4(trajectory[self.eventCounter]))
            self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(trajectory[self.eventCounter].dot(hand_pose_in_obj_frame)))

        self.eventCounter += 1

        return task.again

    def executeEvent(self):
        if len(taskMgr.getDoLaters()) > 0:
            # some tasks is not done yet, so skip this event trigger
            return
        elif self.counter != 0 or self.eventCounter != 0:
            sys.exit()

        taskMgr.doMethodLater(0.1, self.executeAllEvent, 'allevent', extraArgs=[Task.Task(self.executeAllEvent), self.eventQueue])

        ############################################################

        # if self.counter == len(self.eventQueue):
        #     sys.exit()
        # # execute the event according to the counter
        # action, trajectory = self.eventQueue[self.counter]
        # if action == "fingerGait":
        #     self.eventCounter = 0
        #     taskMgr.doMethodLater(0.1, self.fingerGaiting, 'fingerGaitTask', extraArgs=[Task.Task(self.fingerGaiting), trajectory])
        # elif action == "pivot":
        #     self.eventCounter = 0
        #     taskMgr.doMethodLater(0.1, self.pivoting, 'pivotTask', extraArgs=[Task.Task(self.pivoting), trajectory])
        # print("counter", self.counter)
        # self.counter += 1


if __name__ == '__main__':
    base = pandactrl.World(camp=[700,700,700], lookatp=[0,0,0], focusLength=1212)
    this_dir, this_filename = os.path.split(__file__)

    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    # objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "good_book.stl")
    # objpath = os.path.join(this_dir, "objects", "cylinder.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")
    objpath = os.path.join(this_dir, "objects", "Lshape.stl")

    handpkg = fetch_grippernm
    gdb = db.GraspDB()

    # initialize a demo helper
    demoHelper = DemoHelper(objpath, handpkg, base)

    regrasp_planner = ff_regrasp_planner(objpath, handpkg, gdb)

    regrasp_planner.loadDB()

    pickup_planner = StablePickupPlanner(objpath, handpkg, gdb)

    # randomly get a initial grasp
    input_grasp, initial_placement = pickup_planner.randomlyPickOneGrasp()

    liftuppose = pickup_planner.getPrePickupPose(input_grasp[0], input_grasp[1])

    placedownPose, liftuptrajectoryplacement, liftuptrajectorycorners, liftupfingerdirection = pickup_planner.checkWayToPlace(liftuppose, input_grasp[0], input_grasp[1])

    pickup_planner.createPlacementGraph()

    # find the pivot action sequence
    pivotActionSequence = pickup_planner.getPivotPath(initial_placement, input_grasp, placedownPose, input_grasp)
    # [placementid1, placementid2, list of pivot grasps, pivot corner point]

    currentgrasp = input_grasp
    currentplacement = initial_placement.copy()

    # execute a sequence of pivoting actions
    for action in pivotActionSequence:

        placementid1, placementid2, pivotGraspslist, pivotCornerPoint = action

        placement1pose = pg.mat4ToNp(pickup_planner.gdb.loadFreeTabletopPlacementByIds(placementid1))
        placement2pose = pg.mat4ToNp(pickup_planner.gdb.loadFreeTabletopPlacementByIds(placementid2))

        # finger gaiting
        grasp_trajectory = None
        next_grasp = None
        next_jawwidth = None

        for pgl in pivotGraspslist:
            pivotgrasp, pivotgraspJawwidth = pickup_planner.gdb.loadFreeAirGripByIds(pgl)
            grasp_trajectory = regrasp_planner.getTrajectory(currentgrasp[0], pg.mat4ToNp(pivotgrasp), pivotgraspJawwidth, currentplacement, base)
            if not grasp_trajectory == None:
                next_grasp = pg.mat4ToNp(pivotgrasp)
                next_jawwidth = pivotgraspJawwidth
                break

        poseTrajectory = []
        for g in range(len(grasp_trajectory) - 1):
            poseTrajectory.extend(regrasp_planner.getLinearPoseTrajectory(currentplacement.dot(grasp_trajectory[g]), currentplacement.dot(grasp_trajectory[g+1])))

        demoHelper.addEvent("fingerGait", poseTrajectory)

        if pivotCornerPoint[0].shape[0] > 1: # when the pivoting edge is not sharp

            rotatedplacementposelist = [placement1pose] + pivotCornerPoint[2] + [placement2pose]
            for l in range(pivotCornerPoint[0].shape[0]):
                # pivoting
                pivotTrajectory = [currentplacement.dot(np.linalg.inv(rotatedplacementposelist[l])).dot(p) for p in pickup_planner.getPivotTrajectory(rotatedplacementposelist[l], rotatedplacementposelist[l+1], [pivotCornerPoint[0][l], pivotCornerPoint[1]])]
                demoHelper.addEvent("pivot", pivotTrajectory)
                currentgrasp = [next_grasp, next_jawwidth]
                currentplacement = pivotTrajectory[-1]
        else:
            pivotTrajectory = [currentplacement.dot(np.linalg.inv(placement1pose)).dot(p) for p in pickup_planner.getPivotTrajectory(placement1pose, placement2pose, [pivotCornerPoint[0][0], pivotCornerPoint[1]])]
            demoHelper.addEvent("pivot", pivotTrajectory)
            currentgrasp = [next_grasp, next_jawwidth]
            currentplacement = pivotTrajectory[-1]

    # lift up the object
    # move back to the original grasp
    grasp_trajectory = regrasp_planner.getTrajectory(currentgrasp[0], input_grasp[0], input_grasp[1], currentplacement, base)

    if grasp_trajectory != None:
        poseTrajectory = []
        for g in range(len(grasp_trajectory) - 1):
            poseTrajectory.extend(regrasp_planner.getLinearPoseTrajectory(currentplacement.dot(grasp_trajectory[g]), currentplacement.dot(grasp_trajectory[g+1])))

        demoHelper.addEvent("fingerGait", poseTrajectory)

        liftuptrajectoryplacement.append(liftuppose)

        for s in range(len(liftuptrajectorycorners)):
            pivotTrajectory = [currentplacement.dot(np.linalg.inv(liftuptrajectoryplacement[s])).dot(p) for p in pickup_planner.getPivotTrajectory(liftuptrajectoryplacement[s], liftuptrajectoryplacement[s+1], [liftuptrajectorycorners[s][0], liftupfingerdirection])]
            demoHelper.addEvent("pivot", pivotTrajectory)
            currentplacement = pivotTrajectory[-1]
        demoHelper.setObjPose(initial_placement)
        demoHelper.setHandPose(initial_placement.dot(input_grasp[0]), input_grasp[1])

    else:
        print("fail to manipulate the object with some reason")
    
    liftuppose[0][3] += 300
    pickup_planner.showPickUp(base, liftuppose, input_grasp[0], input_grasp[1])


    #####################################################################################
    #                                  show the demo                                    #
    #####################################################################################


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