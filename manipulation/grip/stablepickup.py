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

import networkx as nx
import math
from scipy.spatial import KDTree

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

        # placement graph
        self.PlacementG = nx.Graph()

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
        self.validangle = 0.1

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
        random_index = random.randint(0, len(self.freegripid))
        # random_index = 19
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
        return [self.freegriprotmats[random_index], self.freegripjawwidth[random_index]], dc.strToMat4(result[random_placement_index][0])

    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])


    def checkWayToPlace(self, liftupPose, grasppose, jawwidth):
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

        # find the vertical 2d plane for manipulation
        fingerNormalDirection = self.getPointFromPose(grasppose * liftupPose, Point3(0, 1, 0)) - self.getPointFromPose(grasppose * liftupPose, Point3(0, 0, 0))
        z_axis_rotation = pandageom.cvtMat4(rm.rodrigues([0,0,1], angle_between([fingerNormalDirection[0], fingerNormalDirection[1]], [0, 1])))

        mass2d_x = self.getPointFromPose(liftupPose*z_axis_rotation, Point3(self.objcom[0], self.objcom[1], self.objcom[2]))[0]

        # find the closest ground contact point
        # move the object down a little to cause collision
        liftuppose.setCell(3,2,liftuppose.getCell(3,2) - 1)
        self.npnodeobj.setMat(liftupPose)

        objbullnode = cd.genCollisionMeshMultiNp(self.npnodeobj)
        result = self.bulletworldhp.contactTest(objbullnode)

        ground_touching_point_2d, ground_touching_point_3d = [], []

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
        rotationPoint_2d = ground_touching_point_2d[cloestIndex]
        rotationPoint = ground_touching_point_3d[cloestIndex]

        # find the lowest point on convex on each side
        counterclockwise_rotation_angle_2d, clockwise_rotation_angle_2d = 1.57, 1.57

        for i in range(len(self.objtrimeshconv.faces)):
            for j in range(3):
                vert = self.objtrimeshconv.vertices[self.objtrimeshconv.faces[i][j]]
                vertp = self.getPointFromPose(liftuppose * z_axis_rotation, vert)

                # if the other side of touch point is so close to the rotate point, then ignore it
                if np.linalg.norm([abs(rotationPoint_2d - vertp[0]), vertp[2]]) < 1.0:
                    continue

                if vertp[0] > rotationPoint_2d:
                    temp = np.arctan2(vertp[2], vertp[0] - rotationPoint_2d)
                    if temp < counterclockwise_rotation_angle_2d and temp >= 0.0:
                        counterclockwise_rotation_angle_2d = temp

                elif vertp[0] < rotationPoint_2d:
                    temp = np.arctan2(vertp[2], rotationPoint_2d - vertp[0])
                    if temp < clockwise_rotation_angle_2d and temp >= 0.0:
                        clockwise_rotation_angle_2d = temp

        placedownpose = LMatrix4f(liftuppose)
        placedownpose.setCell(3,0,placedownpose.getCell(3,0) - rotationPoint[0])
        placedownpose.setCell(3,1,placedownpose.getCell(3,1) - rotationPoint[1])
        placedownpose.setCell(3,2,placedownpose.getCell(3,2) - rotationPoint[2])

        if rotationPoint_2d < mass2d_x:
            # rotate to counterclockwise
            rotmat = rm.rodrigues([fingerNormalDirection[0], fingerNormalDirection[1], fingerNormalDirection[2]], counterclockwise_rotation_angle_2d * 180.0 / np.pi)
        elif rotationPoint_2d > mass2d_x:
            #rotate to clockwise
            rotmat = rm.rodrigues([fingerNormalDirection[0], fingerNormalDirection[1], fingerNormalDirection[2]], -clockwise_rotation_angle_2d * 180.0 / np.pi)
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

    def generateManipulationCircle(self, fingerdirections, contactpointInPlane, graspsIdInPlane):
        placement2placement = []
        # according to all possible finger directions, a set of placements can be pivoted by that grasp
        # the placements is in a loop according to the fingertip direction
        for fingerdirection, currentContactPoints, currentGraspsId in zip(fingerdirections, contactpointInPlane, graspsIdInPlane):
            # get all grasps belong to this finger direction
            graspsBelongToCurrentPlane = np.stack(currentContactPoints)[:, :3]

            # ffdirections is the placement direction in the object frame
            # rotate corners are the rotate corners in the object frame
            _, ffdirections, rotateCorners = pg.generateFFPlacement(self.objtrimeshconv, fingerdirection, self.objcom, 0.9)

            ffdirectionsMat = np.stack(ffdirections)

            manipulationPlaneXAxisMat = (np.array([
                    [0, -fingerdirection[2], fingerdirection[1]],
                    [fingerdirection[2], 0, -fingerdirection[0]],
                    [-fingerdirection[1], fingerdirection[0], 0]
                    ]).dot(ffdirectionsMat.transpose())).transpose()

            manipulation_planeMat = np.stack([manipulationPlaneXAxisMat, ffdirectionsMat])

            massCenterInPlaneMat = (np.tensordot(self.objcom, manipulation_planeMat, (0,2)) / np.sum(manipulation_planeMat*manipulation_planeMat, axis=2)).transpose()

            for l in range(len(rotateCorners)):
                rotateCorners[l] = np.array(rotateCorners[l])

            for l in range(len(rotateCorners)):
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
                    left = l
                    right = (l + 1) % len(rotateCorners)
                    # need to verify the grasp has no collision with the ground
                    placement2placement.append([ffdirections[left], ffdirections[right], np.array(currentGraspsId)[valid_common_grasp_bit], (rotateCorners[l][0], fingerdirection)])

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

    def showPivot(self, pivotaction, base):
        """
        this function will visualize the pivot action
        """
        
        placement1, placement2, graspids, pivotPoint = pivotaction

        # calculate the placement poses according to the placement direction
        placementpose1 = self.placementdirection2pose(placement1)
        placementpose2 = self.placementdirection2pose(placement2)

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


    def createPlacementGraph(self, base):
        '''
        create a pivoting graph
        '''

        fingerdirections, contactpointInPlane, graspsIdInPlane = self.generateManipulationPlane()

        placement2placement = self.generateManipulationCircle(fingerdirections, contactpointInPlane, graspsIdInPlane)

        # need to verify the function correctness
        self.showPivot(placement2placement[2], base)







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

    # randomly get a initial grasp
    input_grasp, initial_placement = pickup_planner.randomlyPickOneGrasp()
    # get random placement where the initial grasp is valid

    liftuppose = pickup_planner.getPrePickupPose(input_grasp[0], input_grasp[1])
    placedownPose = pickup_planner.checkWayToPlace(liftuppose, input_grasp[0], input_grasp[1])

    pickup_planner.createPlacementGraph(base)
    # pickup_planner.showPickUp(base, liftuppose, input_grasp[0], input_grasp[1])
    # pickup_planner.showPickUp(base, placedownPose, input_grasp[0], input_grasp[1])
    # pickup_planner.showPickUp(base, initial_placement, input_grasp[0], input_grasp[1])

    


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