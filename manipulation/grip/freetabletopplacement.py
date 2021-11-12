#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
import math
# from manipulation.grip.robotiq85 import rtq85nm
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
from pandaplotutils import pandageom as pg
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from database import dbaccess as db

import matplotlib.pyplot as plt
import geopandas as gpd

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

        # if x < 0.0 or (x == 0.0 and y < 0.0) or (x == 0.0 and y == 0.0 and z < 0.0):
        #     continue

        points.append((x, y, z))

    return points

def point2angle(point):
    return math.atan2(point[2], point[0]), math.atan2(point[1], point[0])

def angle2point(angle):
    t1 = math.tan(angle[0])
    t2 = math.tan(angle[1])
    xvalue = math.sqrt(1.0/(1+t1*t1+t2*t2))
    yvalue = xvalue * t2
    zvalue = xvalue * t1
    return [xvalue, yvalue, zvalue]

def mapParallelDirection(d):
    if d[0] < 0.0 or (d[0] == 0.0 and d[1] < 0.0) or (d[0] == 0.0 and d[1] == 0.0 and d[2] < 0.0):
        return (-d[0], -d[1], -d[2])
    return (d[0], d[1], d[2])


class FreeTabletopPlacement(object):
    """
    manipulation.freetabletopplacement doesn't take into account
    the position and orientation of the object
    it is "free" in position and rotation around z axis
    in contrast, each item in regrasp.tabletopplacements
    has different position and orientation
    it is at a specific pose in the workspace
    To clearly indicate the difference, "free" is attached
    to the front of "freetabletopplacement"
    "s" is attached to the end of "tabletopplacements"
    """

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
        self.bulletworldhp1 = BulletWorld()
        self.bulletworldhplowest = BulletWorld()

        # plane to remove hand
        self.planebullnode = cd.genCollisionPlane(offset=0)
        self.bulletworldhp.attachRigidBody(self.planebullnode)

        self.planebullnode1 = cd.genCollisionPlane(offset=3)
        self.bulletworldhp1.attachRigidBody(self.planebullnode1)

        self.planebullnode2 = cd.genCollisionPlane(offset=30)
        self.bulletworldhplowest.attachRigidBody(self.planebullnode2)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.1])
        # self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])

        # for dbsave
        # each tpsmat4 corresponds to a set of tpsgripcontacts/tpsgripnormals/tpsgripjawwidth list
        self.tpsmat4s = None
        self.tpsgripcontacts = None
        self.tpsgripnormals = None
        self.tpsgripjawwidth = None
        self.tpsdirections = None # a set of possible grasp directions
        self.tpsffplacements = None
        self.tpsffgrips = None
        self.stableplacementdirections = None
        self.tpshasgrasp = None # grasp direction with its all relative grasps
        self.numberOfStable = 0

        # for ocFacetShow
        self.counter = 0

        self.gdb = gdb
        self.loadFreeAirGrip()

    def genFixturelessPlacement(self):

        self.tpsffplacements = []
        self.tpsffgrips = []

        # use a fibonacci method to generate a set of direction in 2d
        angleMap = []
        for t in fibonacci_sphere(642):
            angleMap.append(t)

        self.tpshasgrasp = [[] for _ in range(len(angleMap))] # possible finger directions
        accuracyDirect = np.zeros((len(angleMap), 3))

        # build the kd tree of the direction sets
        self.tpsdirections = KDTree(np.array(angleMap))

        for f in range(len(self.freegripnormals)):
            dis, ind = self.tpsdirections.query(mapParallelDirection(self.freegripnormals[f][0]))
            if len(self.tpshasgrasp[ind]) == 0:
                accuracyDirect[ind][0] = mapParallelDirection(self.freegripnormals[f][0])[0]
                accuracyDirect[ind][1] = mapParallelDirection(self.freegripnormals[f][0])[1]
                accuracyDirect[ind][2] = mapParallelDirection(self.freegripnormals[f][0])[2]
            self.tpshasgrasp[ind].append([self.freegriprotmats[f], self.freegripjawwidth[f], self.freegripid[f], self.freegripnormals[f][0], self.freegripnormals[f][1]])

        fingerdirections = []
        for n in range(len(self.tpshasgrasp)):
            if len(self.tpshasgrasp[n]) > 0:
                fingerdirections.append((accuracyDirect[n], n))
        
        # search all stable placement direction
        stabledirectionbit = np.zeros(len(angleMap))
        for p in range(len(self.stableplacementdirections)):
            sp = (self.stableplacementdirections[p][0], self.stableplacementdirections[p][1], self.stableplacementdirections[p][2])
            dis, ind = self.tpsdirections.query(sp)
            stabledirectionbit[ind] += 1

        # according to all possible finger directions, calculate ff placement
        for d, dn in fingerdirections:
            ffplacements, ffdirections = pg.generateFFPlacement(self.objtrimeshconv, d, self.objcom, 0.9)

            for i in range(len(ffplacements)): # remove the placement which is already stable
                dis, ind = self.tpsdirections.query(ffdirections[i])
                if stabledirectionbit[ind] == 0:
                    self.tpsffplacements.append(pg.cvtMat4np4(ffplacements[i]))
                    self.tpsffgrips.append(dn)

        self.tpsmat4s += self.tpsffplacements

    def loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """
        
        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, handname = self.handname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def loadFreeTabletopPlacement(self):
        """
        load free tabletopplacements

        :return:
        """
        # tpsmat4s = self.gdb.loadFreeTabletopPlacement(self.dbobjname)
        # if tpsmat4s is not None:
        #     self.tpsmat4s = tpsmat4s
        #     return True
        # else:
        #     self.tpsmat4s = []
        #     return False
        self.tpsmat4s = []
        return False

    def removebadfacets(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """
        self.tpsmat4s = []
        self.stableplacementdirections = []

        for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworldray.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            self.bulletworldray.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                     facetinterpnt, facetnormal)
                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                if dist2p/dist2c >= doverh:
                    # hit and stable
                    self.tpsmat4s.append(pg.cvtMat4np4(facetmat4))
                    # record the stable placement direction
                    self.stableplacementdirections.append((np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))/dist2c)

        self.numberOfStable = len(self.tpsmat4s)

    def gentpsgrip(self, base):
        """
        Originally the code of this function is embedded in the removebadfacet function
        It is separated on 20170608 to enable common usage of placements for different hands

        :return:

        author: weiwei
        date: 20170608
        """

        self.tpsgripcontacts = []
        self.tpsgripnormals = []
        self.tpsgriprotmats = []
        self.tpsgripjawwidth = []
        # the id of the grip in freeair
        self.tpsgripidfreeair = []

        for i in range(len(self.tpsmat4s)):
            self.tpsgripcontacts.append([])
            self.tpsgripnormals.append([])
            self.tpsgriprotmats.append([])
            self.tpsgripjawwidth.append([])
            self.tpsgripidfreeair.append([])
            if i >= self.numberOfStable:
                ii = i - self.numberOfStable
                for rotmat, jawwidth, gripid, cctn0, cctn1 in self.tpshasgrasp[self.tpsffgrips[ii]]:
                    tpsgriprotmat = rotmat * self.tpsmat4s[i]
                    
                    # check if the hand collide with tabletop
                    # tmprtq85 = self.rtq85hnd
                    tmphnd = self.hand
                    # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
                    initmat = tmphnd.getMat()
                    initjawwidth = tmphnd.jawwidth
                    # open the hand to ensure it doesnt collide with surrounding obstacles
                    # tmprtq85.setJawwidth(self.freegripjawwidth[j])
                    tmphnd.setJawwidth(jawwidth) #self.freegripjawwidth[j])
                    tmphnd.setMat(pandanpmat4 = tpsgriprotmat)
                    # add hand model to bulletworld
                    hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
                    hndbull_without_fingers_node = cd.genCollisionMeshMultiNp(tmphnd.palmnp)
                    result0 = self.bulletworldhp.contactTest(hndbullnode)
                    result1 = self.bulletworldhp1.contactTest(hndbullnode)
                    result2 = self.bulletworldhplowest.contactTest(hndbull_without_fingers_node)
                    # print result.getNumContacts()
                    if not result0.getNumContacts() and not result1.getNumContacts() and not result2.getNumContacts():
                        self.tpsgriprotmats[-1].append(tpsgriprotmat)
                        # cct0 = self.tpsmat4s[i].xformPoint(self.freegripcontacts[j][0])
                        # cct1 = self.tpsmat4s[i].xformPoint(self.freegripcontacts[j][1])
                        # cct0 = self.freegripcontacts[j][0]
                        # cct1 = self.freegripcontacts[j][1]
                        cct0, cct1 = tmphnd.getFingerTips()
                        cct0 = self.getPointFromPose(tpsgriprotmat, cct0)
                        cct1 = self.getPointFromPose(tpsgriprotmat, cct1)
                        self.tpsgripcontacts[-1].append([cct0, cct1])
                        # cctn0 = self.tpsmat4s[i].xformVec(self.freegripnormals[j][0])
                        # cctn1 = self.tpsmat4s[i].xformVec(self.freegripnormals[j][1])
                        # cctn0 = self.freegripnormals[j][0]
                        # cctn1 = self.freegripnormals[j][1]
                        self.tpsgripnormals[-1].append([cctn0, cctn1])
                        self.tpsgripjawwidth[-1].append(jawwidth)#self.freegripjawwidth[j])
                        self.tpsgripidfreeair[-1].append(gripid)#self.freegripid[j])
                    tmphnd.setMat(pandanpmat4 = initmat)
                    tmphnd.setJawwidth(initjawwidth)
            else:
                for j, rotmat in enumerate(self.freegriprotmats):

                    tpsgriprotmat = rotmat * self.tpsmat4s[i]
                    
                    # check if the hand collide with tabletop
                    # tmprtq85 = self.rtq85hnd
                    tmphnd = self.hand
                    # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
                    initmat = tmphnd.getMat()
                    initjawwidth = tmphnd.jawwidth
                    # open the hand to ensure it doesnt collide with surrounding obstacles
                    # tmprtq85.setJawwidth(self.freegripjawwidth[j])
                    tmphnd.setJawwidth(self.freegripjawwidth[j])
                    tmphnd.setMat(pandanpmat4 = tpsgriprotmat)
                    # add hand model to bulletworld
                    hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
                    hndbull_without_fingers_node = cd.genCollisionMeshMultiNp(tmphnd.palmnp)
                    result0 = self.bulletworldhp.contactTest(hndbullnode)
                    result1 = self.bulletworldhp1.contactTest(hndbullnode)
                    result2 = self.bulletworldhplowest.contactTest(hndbull_without_fingers_node)
                    if not result0.getNumContacts() and not result1.getNumContacts() and not result2.getNumContacts():
                        self.tpsgriprotmats[-1].append(tpsgriprotmat)
                        # cct0 = self.tpsmat4s[i].xformPoint(self.freegripcontacts[j][0])
                        # cct1 = self.tpsmat4s[i].xformPoint(self.freegripcontacts[j][1])
                        # cct0 = self.freegripcontacts[j][0]
                        # cct1 = self.freegripcontacts[j][1]
                        cct0, cct1 = tmphnd.getFingerTips()
                        cct0 = self.getPointFromPose(tpsgriprotmat, cct0)
                        cct1 = self.getPointFromPose(tpsgriprotmat, cct1)
                        self.tpsgripcontacts[-1].append([cct0, cct1])
                        # cctn0 = self.tpsmat4s[i].xformVec(self.freegripnormals[j][0])
                        # cctn1 = self.tpsmat4s[i].xformVec(self.freegripnormals[j][1])
                        cctn0 = self.freegripnormals[j][0]
                        cctn1 = self.freegripnormals[j][1]
                        self.tpsgripnormals[-1].append([cctn0, cctn1])
                        self.tpsgripjawwidth[-1].append(self.freegripjawwidth[j])
                        self.tpsgripidfreeair[-1].append(self.freegripid[j])
                    tmphnd.setMat(pandanpmat4 = initmat)
                    tmphnd.setJawwidth(initjawwidth)

    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])

    def saveToDB(self):
        """
        save freetabletopplacement

        manipulation.freetabletopplacement doesn't take into account the position and orientation of the object
        it is "free" in position and rotation around z axis
        in contrast, each item in regrasp.tabletopplacements has different position and orientation
        it is at a specific pose in the workspace
        To clearly indicate the difference, "free" is attached to the front of "freetabletopplacement"
        "s" is attached to the end of "tabletopplacements"

        :param discretesize:
        :param gdb:
        :return:

        author: weiwei
        date: 20170111
        """

        # save freetabletopplacement
        sql = "SELECT * FROM freetabletopplacement,object WHERE freetabletopplacement.idobject = object.idobject \
                AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) == 0:
            # the fretabletopplacements for the self.dbobjname is not saved
            sql = "INSERT INTO freetabletopplacement(rotmat, idobject, placement) VALUES "
            for i in range(len(self.tpsmat4s)):
                sql += "('%s', (SELECT idobject FROM object WHERE name LIKE '%s'), %d), " % \
                       (dc.mat4ToStr(self.tpsmat4s[i]), self.dbobjname, 0 if i < self.numberOfStable else 1)
            sql = sql[:-2] + ";"
            self.gdb.execute(sql)
        else:
            print "Freetabletopplacement already exist!"

        # save freetabletopgrip
        idhand = gdb.loadIdHand(self.handname)
        sql = "SELECT * FROM freetabletopgrip,freetabletopplacement,freeairgrip,object WHERE \
                freetabletopgrip.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement AND \
                freetabletopgrip.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                freetabletopplacement.idobject = object.idobject AND \
                object.name LIKE '%s' AND freeairgrip.idhand = %d" % (self.dbobjname, idhand)
        result = self.gdb.execute(sql)
        if len(result) == 0:
            for i in range(len(self.tpsmat4s)):
                sql = "SELECT freetabletopplacement.idfreetabletopplacement FROM freetabletopplacement,object WHERE \
                        freetabletopplacement.rotmat LIKE '%s' AND \
                        object.name LIKE '%s'" % (dc.mat4ToStr(self.tpsmat4s[i]), self.dbobjname)
                result = self.gdb.execute(sql)[0]
                # print result
                if len(result) != 0:
                    idfreetabletopplacement = result[0]
                    # note self.tpsgriprotmats[i] might be empty (no cd-free grasps)
                    if len(self.tpsgriprotmats[i]) != 0:
                        sql = "INSERT INTO freetabletopgrip(contactpoint0, contactpoint1, contactnormal0, contactnormal1, \
                                rotmat, jawwidth, idfreetabletopplacement, idfreeairgrip) VALUES "
                        for j in range(len(self.tpsgriprotmats[i])):
                            cct0 = self.tpsgripcontacts[i][j][0]
                            cct1 = self.tpsgripcontacts[i][j][1]
                            cctn0 = self.tpsgripnormals[i][j][0]
                            cctn1 = self.tpsgripnormals[i][j][1]
                            sql += "('%s', '%s', '%s', '%s', '%s', '%s', %d, %d), " % \
                                   (dc.v3ToStr(cct0), dc.v3ToStr(cct1), dc.v3ToStr(cctn0), dc.v3ToStr(cctn1), \
                                    dc.mat4ToStr(self.tpsgriprotmats[i][j]), str(self.tpsgripjawwidth[i][j]), \
                                    idfreetabletopplacement, self.tpsgripidfreeair[i][j])
                        sql = sql[:-2] + ";"
                        self.gdb.execute(sql)
        else:
            print "Freetabletopgrip already exist!"

    def removebadfacetsshow(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """

        plotoffsetfp = 10
        # print self.counter

        if self.counter < len(self.ocfacets):
            i = self.counter
        # for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworldray.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            self.bulletworldray.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()
                pg.plotArrow(base.render, spos=self.objcom,
                             epos = self.objcom+self.ocfacetnormals[i], length=100)

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                   facetinterpnt, facetnormal)
                for j in range(len(bdverts3d)-1):
                    spos = bdverts3d[j]
                    epos = bdverts3d[j+1]
                    pg.plotStick(base.render, spos, epos, thickness = 1, rgba=[.5,.5,.5,1])

                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                if dist2p/dist2c < doverh:
                    print "not stable"
                    # return
                else:
                    print dist2p/dist2c
                    pol_ext = LinearRing(bdverts2d)
                    d = pol_ext.project(apntpnt)
                    p = pol_ext.interpolate(d)
                    closest_point_coords = list(p.coords)[0]
                    closep = np.array([closest_point_coords[0], closest_point_coords[1], 0])
                    closep3d = rm.transformmat4(rm.homoinverse(facetmat4), closep)[:3]
                    pg.plotDumbbell(base.render, spos=facetinterpnt, epos=closep3d, thickness=1.5, rgba=[0,0,1,1])

                    for j in range(len(bdverts3d)-1):
                        spos = bdverts3d[j]
                        epos = bdverts3d[j+1]
                        pg.plotStick(base.render, spos, epos, thickness = 1.5, rgba=[0,1,0,1])

                    # geomoff = pg.packpandageom(self.objtrimeshconv.vertices +
                    #                                np.tile(plotoffsetfp * self.ocfacetnormals[i],
                    #                                        [self.objtrimeshconv.vertices.shape[0], 1]),
                    #                         self.objtrimeshconv.face_normals[self.ocfacets[i]],
                    #                         self.objtrimeshconv.faces[self.ocfacets[i]])
                    #
                    # nodeoff = GeomNode('supportfacet')
                    # nodeoff.addGeom(geomoff)
                    # staroff = NodePath('supportfacet')
                    # staroff.attachNewNode(nodeoff)
                    # staroff.setColor(Vec4(1,0,1,1))
                    # staroff.setTransparency(TransparencyAttrib.MAlpha)
                    # staroff.setTwoSided(True)
                    # staroff.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter=0

    def grpshow(self, base):

        sql = "SELECT freetabletopplacement.idfreetabletopplacement, freetabletopplacement.rotmat \
                       FROM freetabletopplacement,object WHERE \
                       freetabletopplacement.idobject = object.idobject AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) != 0:
            idfreetabletopplacement = int(result[3][0])
            objrotmat  = dc.strToMat4(result[3][1])
            # show object
            geom = pg.packpandageom(self.objtrimesh.vertices,
                                    self.objtrimesh.face_normals,
                                    self.objtrimesh.faces)
            node = GeomNode('obj')
            node.addGeom(geom)
            star = NodePath('obj')
            star.attachNewNode(node)
            star.setColor(Vec4(.77,0.67,0,1))
            star.setTransparency(TransparencyAttrib.MAlpha)
            star.setMat(objrotmat)
            star.reparentTo(base.render)
            sql = "SELECT freetabletopgrip.rotmat, freetabletopgrip.jawwidth FROM freetabletopgrip WHERE \
                                freetabletopgrip.idfreetabletopplacement=%d" % idfreetabletopplacement
            result = self.gdb.execute(sql)
            for resultrow in result:
                hndrotmat = dc.strToMat4(resultrow[0])
                hndjawwidth = float(resultrow[1])
                # show grasps
                # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, .1])
                tmprtq85 = fetch_grippernm.Fetch_gripperNM(hndcolor=[1, 0, 0, .1])
                tmprtq85.setMat(pandanpmat4 = hndrotmat)
                tmprtq85.setJawwidth(hndjawwidth)
                # tmprtq85.setJawwidth(80)
                tmprtq85.reparentTo(base.render)

    def showOnePlacementAndAssociatedGrips(self, base):
        """
        show one placement and its associated grasps
        :param base:
        :return:
        """

        for i in range(len(self.tpsmat4s)):
            if i == 0:
                objrotmat  = self.tpsmat4s[i]
                # objrotmat.setRow(0, -objrotmat.getRow3(0))
                rotzmat = Mat4.rotateMat(0, Vec3(0,0,1))
                objrotmat = objrotmat*rotzmat
                # show object
                geom = pg.packpandageom(self.objtrimesh.vertices,
                                        self.objtrimesh.face_normals,
                                        self.objtrimesh.faces)
                node = GeomNode('obj')
                node.addGeom(geom)
                star = NodePath('obj')
                star.attachNewNode(node)
                star.setColor(Vec4(.7,0.3,0,1))
                star.setTransparency(TransparencyAttrib.MAlpha)
                star.setMat(objrotmat)
                star.reparentTo(base.render)
                for j in range(len(self.tpsgriprotmats[i])):
                    hndrotmat = self.tpsgriprotmats[i][j]
                    hndjawwidth = self.tpsgripjawwidth[i][j]
                    # show grasps
                    tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
                    tmphnd.setMat(pandanpmat4 = hndrotmat)
                    tmphnd.setJawwidth(hndjawwidth)
                    # tmphnd.setJawwidth(0)
                    # tmprtq85.setJawwidth(80)
                    tmphnd.reparentTo(base.render)
    
    def showAllFFPlacement(self, base):
        distanceBetweenObjects = 300

        numberOfPlacements = len(self.tpsffplacements)
        print "number of ff placements = ", numberOfPlacements

        tableSideLength = int(math.sqrt(numberOfPlacements)) + 1
        tableHeight = 0
        if(numberOfPlacements % tableSideLength == 0.0):
            tableHeight = int(numberOfPlacements/tableSideLength)
        else:
            tableHeight = int(numberOfPlacements/tableSideLength) + 1
        placementTable = np.zeros((tableHeight, tableSideLength, 2))

        for i in range(placementTable.shape[0]):
            for j in range(placementTable.shape[1]):
                placementTable[i][j][0] = i - int(placementTable.shape[0] / 2)
                placementTable[i][j][1] = j - int(placementTable.shape[1] / 2)

        
        placementTable = placementTable.reshape(-1,2)

        for i in range(len(self.tpsffplacements)):
            tx, ty = placementTable[i]

            objrotmat = self.tpsffplacements[i]

            objrotmat.setCell(3,0,tx * distanceBetweenObjects)
            objrotmat.setCell(3,1,ty * distanceBetweenObjects)

            rotzmat = Mat4.rotateMat(0, Vec3(0,0,1))
            objrotmat = objrotmat*rotzmat
            # show object
            geom = pg.packpandageom(self.objtrimesh.vertices,
                                    self.objtrimesh.face_normals,
                                    self.objtrimesh.faces)
            node = GeomNode('obj')
            node.addGeom(geom)
            star = NodePath('obj')
            star.attachNewNode(node)
            star.setColor(Vec4(.7,0.8,0,1))
            # star.setTransparency(TransparencyAttrib.MAlpha)
            star.setMat(objrotmat)
            star.reparentTo(base.render)

    def showAllPlacementAndAssociatedGrips(self, base):
        """
        show all placement and its associated grasps
        :param base:
        :return:
        """

        distanceBetweenObjects = 350

        numberOfPlacements = len(self.tpsmat4s)
        print "number of placements = ", numberOfPlacements
        tableSideLength = int(math.sqrt(numberOfPlacements)) + 1
        tableHeight = 0
        if(numberOfPlacements % tableSideLength == 0.0):
            tableHeight = int(numberOfPlacements/tableSideLength)
        else:
            tableHeight = int(numberOfPlacements/tableSideLength) + 1
        placementTable = np.zeros((tableHeight, tableSideLength, 2))

        for i in range(placementTable.shape[0]):
            for j in range(placementTable.shape[1]):
                placementTable[i][j][0] = i - int(placementTable.shape[0] / 2)
                placementTable[i][j][1] = j - int(placementTable.shape[1] / 2)
        placementTable = placementTable.reshape(-1,2)

        # placementTable = np.zeros((numberOfPlacements, 2))
        # for i in range(numberOfPlacements):
        #     placementTable[i][0] = i - int(numberOfPlacements/2)
        #     placementTable[i][1] = 0
        

        for i in range(len(self.tpsmat4s)):


            tx, ty = placementTable[i]
            objrotmat = self.tpsmat4s[i]
            objrotmat.setCell(3,0,tx * distanceBetweenObjects)
            objrotmat.setCell(3,1,ty * distanceBetweenObjects)

            rotzmat = Mat4.rotateMat(0, Vec3(0,0,1))
            objrotmat = objrotmat*rotzmat
            # show object
            geom = pg.packpandageom(self.objtrimesh.vertices,
                                    self.objtrimesh.face_normals,
                                    self.objtrimesh.faces)
            node = GeomNode('obj')
            node.addGeom(geom)
            star = NodePath('obj')
            star.attachNewNode(node)
            if i < self.numberOfStable:
                star.setColor(Vec4(.7,0.3,0,1))
            else:
                star.setColor(Vec4(.7,0.8,0,1))
            star.setTransparency(TransparencyAttrib.MAlpha)
            star.setMat(objrotmat)
            star.reparentTo(base.render)

            # show the gripers
            for j in range(len(self.tpsgriprotmats[i])):
                
                hndrotmat = self.tpsgriprotmats[i][j]
                hndrotmat.setCell(3,0,tx * distanceBetweenObjects + hndrotmat.getCell(3,0))
                hndrotmat.setCell(3,1,ty * distanceBetweenObjects + hndrotmat.getCell(3,1))
                hndjawwidth = self.tpsgripjawwidth[i][j]
                # show grasps
                tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
                tmphnd.setMat(pandanpmat4 = hndrotmat)
                tmphnd.setJawwidth(hndjawwidth)
                # tmphnd.setJawwidth(0)
                # tmprtq85.setJawwidth(80)
                
                cct0 = self.tpsgripcontacts[i][j][0]
                cct1 = self.tpsgripcontacts[i][j][1]

                pandageom.plotSphere(base.render, pos=Point3(cct0[0] + tx * distanceBetweenObjects, cct0[1] + ty * distanceBetweenObjects, cct0[2]), radius=5, rgba=Vec4(0,1,0,1))
                pandageom.plotSphere(base.render, pos=Point3(cct1[0] + tx * distanceBetweenObjects, cct1[1] + ty * distanceBetweenObjects, cct1[2]), radius=5, rgba=Vec4(1,0,0,1))
                # pandageom.plotSphere(base.render, pos=Point3(cct0[0], cct0[1] , cct0[2]), radius=5, rgba=Vec4(0,1,0,1))
                # pandageom.plotSphere(base.render, pos=Point3(cct1[0], cct1[1] , cct1[2]), radius=5, rgba=Vec4(1,0,0,1))
                tmphnd.reparentTo(base.render)


    def ocfacetshow(self, base):
        print self.objcom

        npf = base.render.find("**/supportfacet")
        if npf:
            npf.removeNode()

        plotoffsetfp = 10

        print self.counter
        print len(self.ocfacets)
        if self.counter < len(self.ocfacets):
            geom = pandageom.packpandageom(self.objtrimeshconv.vertices+
                                           np.tile(plotoffsetfp*self.ocfacetnormals[self.counter],
                                                   [self.objtrimeshconv.vertices.shape[0],1]),
                                           self.objtrimeshconv.face_normals[self.ocfacets[self.counter]],
                                           self.objtrimeshconv.faces[self.ocfacets[self.counter]])
            # geom = pandageom.packpandageom(self.objtrimeshconv.vertices,
            #                                self.objtrimeshconv.face_normals,
            #                                self.objtrimeshconv.faces)
            node = GeomNode('supportfacet')
            node.addGeom(geom)
            star = NodePath('supportfacet')
            star.attachNewNode(node)
            star.setColor(Vec4(1,0,1,1))
            star.setTransparency(TransparencyAttrib.MAlpha)
            star.setTwoSided(True)
            star.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter = 0

if __name__ == '__main__':

    base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "sandpart.stl")
    # objpath = os.path.join(this_dir, "objects", "ttube.stl")
    # objpath = os.path.join(this_dir, "objects", "tool.stl")
    # objpath = os.path.join(this_dir, "objects", "tool2.stl")
    # objpath = os.path.join(this_dir, "objects", "planewheel.stl")
    # objpath = os.path.join(this_dir, "objects", "planelowerbody.stl")
    # objpath = os.path.join(this_dir, "objects", "planefrontstay.stl")
    # objpath = os.path.join(this_dir, "objects", "planerearstay.stl")
    # objpath = os.path.join(this_dir, "objects", "planerearstay2.stl")
    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "good_book.stl")
    # objpath = os.path.join(this_dir, "objects", "cylinder.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")
    # objpath = os.path.join(this_dir, "objects", "Lshape.stl")

    # from manipulation.grip.hrp5three import hrp5threenm
    # handpkg = hrp5threenm
    # handpkg = rtq85nm
    handpkg = fetch_grippernm
    gdb = db.GraspDB()
    tps = FreeTabletopPlacement(objpath, handpkg, gdb)

    # objpath0 = os.path.join(this_dir, "objects", "ttube.stl")
    # objpath1 = os.path.join(this_dir, "objects", "tool.stl")
    # objpath2 = os.path.join(this_dir, "objects", "planewheel.stl")
    # objpath3 = os.path.join(this_dir, "objects", "planelowerbody.stl")
    # objpath4 = os.path.join(this_dir, "objects", "planefrontstay.stl")
    # objpath5 = os.path.join(this_dir, "objects", "planerearstay.stl")
    # objpaths = [objpath0, objpath1, objpath2, objpath3, objpath4, objpath5]
    # import time
    # fo = open("foo.txt", "w")
    # for objpath in objpaths:
    #     tic = time.clock()
    #     tps = FreeTabletopPlacement(objpath, gdb)
    #     tps.removebadfacets(base, doverh=.2)
    #     toc = time.clock()
    #     print toc-tic
    #     fo.write(os.path.basename(objpath)+' '+str(toc-tic)+'\n')
    # fo.close()

    # # plot obj and its convexhull
    # geom = pandageom.packpandageom(tps.objtrimesh.vertices,
    #                                tps.objtrimesh.face_normals,
    #                                tps.objtrimesh.faces)
    # node = GeomNode('obj')
    # node.addGeom(geom)
    # star = NodePath('obj')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,1,0,1))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    # geom = pandageom.packpandageom(tps.objtrimeshconv.vertices,
    #                                tps.objtrimeshconv.face_normals,
    #                                tps.objtrimeshconv.faces)
    # node = GeomNode('objconv')
    # node.addGeom(geom)
    # star = NodePath('objconv')
    # star.attachNewNode(node)
    # star.setColor(Vec4(0, 1, 0, .3))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    # pg.plotSphere(base.render, pos=tps.objcom, radius=10, rgba=[1,0,0,1])
    # def updateshow(task):
    #     # tps.ocfacetshow(base)
    #     tps.removebadfacetsshow(base, doverh=.1)
    #     return task.again
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont
    
    if tps.loadFreeTabletopPlacement():
        pass
    else:
        tps.removebadfacets(base, doverh=.15)

    tps.genFixturelessPlacement()
    tps.gentpsgrip(base)
    tps.saveToDB()
    
    bullcldrnp = base.render.attachNewNode("bulletcollider")
    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(True)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    debugNP.show()
    
    tps.bulletworldhp.setDebugNode(debugNP.node())
    
    taskMgr.add(updateworld, "updateworld", extraArgs=[tps.bulletworldhp], appendTask=True)

    # tps.grpshow(base)
    # tps.showOnePlacementAndAssociatedGrips(base)
    tps.showAllPlacementAndAssociatedGrips(base)
    # tps.showAllFFPlacement(base)

    # print "check ", tps.planebullnode.findAllMatches("**/+GeomNode")

    base.run()