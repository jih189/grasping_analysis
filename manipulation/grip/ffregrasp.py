#!/usr/bin/python
import os

import numpy as np
from numpy.lib.function_base import angle
import networkx as nx

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
from scipy.spatial.transform import Rotation as R
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.special import softmax

from trimesh import geometry as trigeom

from direct.task import Task
from tqdm import tqdm

class dexterousManipulationGraph:
    def __init__(self, angle_range, edges, is_converted=False):
        
        # this function will read a set of possible angles and edges to build the dexterous manipulation graph
        self.nodes = None # [ [g0,g5], [g1,g2],[g3,g4,gx...], ....]
        self.Graph = nx.Graph()
        self.convert_to_networkX(angle_range, edges, is_converted) #Sets nodes and creates a graph with edges using nodes
        
    # this function will read in the angle range and edges, then convert to a form used by networkX
    def convert_to_networkX(self, angle_range, edges, is_converted):
        """
        when we create the dmg, given angle range is in following format:
        [[anglerange1, anglerange2], [anglerange3], ]
        each element in list is a list of angle range sharing the same contact point.

        """

        if not is_converted:
            self.nodes = [item for sublist in angle_range for item in sublist]
            for i in range(len(self.nodes)):
                self.Graph.add_node(i)
            
            helper_nodes_cnt = []
            tmp = 0
            for i in range(len(angle_range)):
                helper_nodes_cnt.append(tmp)
                tmp += len(angle_range[i])
            for i in range(0,len(edges)):
                e1 = helper_nodes_cnt[ edges[i][0][0] ] + edges[i][0][1]
                e2 = helper_nodes_cnt[ edges[i][1][0] ] + edges[i][1][1]
                c = edges[i][2]
                self.Graph.add_edge(e1,e2, connectgrasp=c, edgenode = (e1, e2))
        else:
            self.nodes = angle_range
            for i in range(0,len(edges)):
                self.Graph.add_edge(edges[i][0], edges[i][1], connectgrasp=edges[i][2], edgenode = (edges[i][0], edges[i][1]))


    def getEdges(self):
        return self.Graph.edges.data()

    def getAngleRange(self):
        return self.nodes
    
    def insert_init_grasp_2_DMG(self, clst_ang_range_idx, grasp, c_grasp_idx):

        self.nodes.append([grasp])
        grasp_idx = len(self.nodes) - 1
        self.start_grasp_and_idx = (grasp, grasp_idx)

        self.Graph.add_node(grasp_idx)
        self.Graph.add_edge(clst_ang_range_idx,grasp_idx, connectgrasp=(c_grasp_idx,0), edgenode = (clst_ang_range_idx, grasp_idx))


    def insert_end_grasp_2_DMG(self, clst_ang_range_idx, grasp, c_grasp_idx):
        self.nodes.append([grasp])
        grasp_idx = len(self.nodes) - 1
        self.end_grasp_and_idx = (grasp, grasp_idx)

        self.Graph.add_node(grasp_idx)
        self.Graph.add_edge(clst_ang_range_idx,grasp_idx, connectgrasp=(c_grasp_idx,0), edgenode = (clst_ang_range_idx, grasp_idx))

    def remove_init_and_end_grasp_from_DMG(self):
        """
        remove the initial and target nodes from the DMG
        """
        # pop end grasp
        self.nodes.pop()
        self.Graph.remove_node(self.end_grasp_and_idx[1])
        # pop start grasp
        self.nodes.pop()
        self.Graph.remove_node(self.start_grasp_and_idx[1])

    def is_trajectory_possible(self, st_grasp_idx,ed_grasp_idx):
        """Checks if grasp points have a possible path"""
        try:
            return nx.shortest_path(self.Graph,st_grasp_idx,ed_grasp_idx)
        except Exception as e:
            print(e)
            return None
       
    def getGraspTrajectory(self):
        # this function will insert both start grasp and end grasp into the dexterous manipulation graph
        # and plan for the grasp trajectory from start grasp to end grasp
        if not self.start_grasp_and_idx or not self.end_grasp_and_idx:
            print("Please give a start and/or End grasp")
            exit()

        path_nodes = self.is_trajectory_possible(self.start_grasp_and_idx[1],self.end_grasp_and_idx[1])

        if path_nodes == None:
            return None
        
        pos_path = []
        in_ang_idx = None
        out_ang_idx = None

        for i in range(0, len(path_nodes)-1):
            currnt_node = path_nodes[i]
            next_node = path_nodes[i+1]
            
            edgenode = self.Graph.get_edge_data(currnt_node,next_node)['edgenode']
            connect_grasp = self.Graph.get_edge_data(currnt_node, next_node)['connectgrasp']
            if edgenode[0] == currnt_node:
                currnt_pos = self.nodes[edgenode[0]][connect_grasp[0]]
                next_pos = self.nodes[edgenode[1]][connect_grasp[1]]
                if not in_ang_idx:
                    in_ang_idx = (edgenode[1], connect_grasp[1])
                elif not out_ang_idx:
                    in_ang_idx_cp = (edgenode[1], connect_grasp[1])
                    out_ang_idx = (edgenode[0], connect_grasp[0])
            else:
                currnt_pos = self.nodes[edgenode[1]][connect_grasp[1]]
                next_pos = self.nodes[edgenode[0]][connect_grasp[0]]
                if not in_ang_idx:
                    in_ang_idx = (edgenode[0], connect_grasp[0])
                elif not out_ang_idx:
                    in_ang_idx_cp = (edgenode[0], connect_grasp[0])
                    out_ang_idx = (edgenode[1], connect_grasp[1])

            
            if i != 0:
                
                if in_ang_idx[1] < out_ang_idx[1]:
                    ang_range_pos = self.nodes[in_ang_idx[0]][in_ang_idx[1]+1: out_ang_idx[1]]
                else:
                    ang_range_pos = self.nodes[in_ang_idx[0]][out_ang_idx[1]+1: in_ang_idx[1]]
                    ang_range_pos.reverse()
                for pos in ang_range_pos:
                    pos_path.append(pos)
                in_ang_idx = in_ang_idx_cp
                out_ang_idx = None
                pos_path.append(currnt_pos)
            else:
                pos_path.append(currnt_pos)
                
            
            pos_path.append(next_pos)
        return pos_path


class ff_regrasp_planner(object):

    def __init__(self, objpath, handpkg, gdb):
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # use two bulletworld, one for the ray, the other for the tabletop
        self.bulletworldhp = BulletWorld()

        # plane to remove hand
        self.planebullnode = cd.genCollisionPlane(offset=0)
        self.bulletworldhp.attachRigidBody(self.planebullnode)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = self.handpkg.newHandNM(hndcolor=[0,0,0,0])

        self.regrasp_graph = None

        self.gdb = gdb
        # self.loadFreeAirGrip()

        self.tpsmat4s = None
        self.placementid = None
        self.gripsOfPlacement = None
        self.loadFreeTabletopPlacement()
        self.loadFreeAirGripForPlacement()

        self.bulletObj = BulletWorld()
        self.star = None

    # this function will save the dmg table into database
    def saveToDB(self):
        # save to database
        gdb = db.GraspDB()
        objectId = None

        sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % self.dbobjname
        result = gdb.execute(sql)
        if not result:
            print("please add the object name to table first!!!")
            return
        else:
            objectId = int(result[0][0]) 

        # get current dmg index
        sql = "SELECT iddmg FROM dmgs"
        result = gdb.execute(sql)
        currentIndexOfDmgs = len(result) + 1

        # get current angle range index
        sql = "SELECT idanglerange FROM angleranges"
        result = gdb.execute(sql)
        currentIndexOfAngleRange = len(result) + 1

        # get current grasp index
        sql = "SELECT idgrasp FROM grasps"
        result = gdb.execute(sql)
        currentIndexOfGrasps = len(result) + 1

        sql = "SELECT * FROM dmgs, object WHERE dmgs.idobject = object.idobject AND \
                object.name LIKE '%s'" % (self.dbobjname)
        result = gdb.execute(sql)

        if not result:
            print("save the DMG table to database")
            for p in tqdm(range(len(self.regrasp_graph))):
                for i_plane in range(len(self.regrasp_graph[p])):
                    dmg = self.regrasp_graph[p][i_plane][1]
                    angleRanges = dmg.getAngleRange()
                    angleRangeIdInDB = []
                    for i_angleRange in range(len(angleRanges)):
                        # save grasps of current anglerange
                        graspOrder = []
                        for i_grasp in range(len(angleRanges[i_angleRange])):
                            grasp = angleRanges[i_angleRange][i_grasp]
                            graspOrder.append(currentIndexOfGrasps)
                            sql = "INSERT INTO grasps(idgrasp, idanglerange, pose) \
                                VALUES('%d', '%d', '%s')" % \
                                (currentIndexOfGrasps, currentIndexOfAngleRange, dc.mat4ToStr(pg.cvtMat4np4(grasp)))
                            gdb.execute(sql)
                            currentIndexOfGrasps += 1

                        angleRangeIdInDB.append(currentIndexOfAngleRange)

                        # save angleRange
                        sql = "INSERT INTO angleranges(idanglerange, iddmg, grasporder) \
                            VALUES('%d', '%d', '%s')" % \
                            (currentIndexOfAngleRange, currentIndexOfDmgs, str(graspOrder))
                        gdb.execute(sql)
                        currentIndexOfAngleRange += 1

                    # need to save angle range edges
                    dmgedges = dmg.getEdges()
                    for _,_,e in dmgedges:
                        connectgrasp1 = e['connectgrasp'][0]
                        connectgrasp2 = e['connectgrasp'][1]
                        endnode1 = angleRangeIdInDB[e['edgenode'][0]]
                        endnode2 = angleRangeIdInDB[e['edgenode'][1]]

                        sql = "INSERT INTO dmgedges(iddmg, idanglerange1, idanglerange2, connectid1, connectid2) \
                            VALUES('%d', '%d', '%d', '%d', '%d')" % \
                            (currentIndexOfDmgs, endnode1, endnode2, connectgrasp1, connectgrasp2)
                        gdb.execute(sql)

                    # save the dmg
                    sql = "INSERT INTO dmgs(iddmg, idobject, placementpose, placementid, planevector) \
                       VALUES('%d', '%d', '%s', '%d', '%s')" % \
                      (currentIndexOfDmgs, objectId, dc.mat4ToStr(self.tpsmat4s[p]), self.placementid[p], dc.v6ToStr(self.regrasp_graph[p][i_plane][0]))
                    gdb.execute(sql)
                    currentIndexOfDmgs += 1

        else:
            print("Dmg already saved or duplicated filename!")

    # this function will load the dmg table from databse
    def loadDB(self):
        # save to database
        gdb = db.GraspDB()
        objectId = None

        sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % self.dbobjname
        result = gdb.execute(sql)
        if not result:
            print("No such object exist in database!!!")
            return
        else:
            objectId = int(result[0][0]) 

        sql = "SELECT * FROM dmgs, object WHERE dmgs.idobject = object.idobject AND \
            object.name LIKE '%s'" % (self.dbobjname)
        result = gdb.execute(sql)

        if result:
            self.regrasp_graph = [[] for _ in range(len(self.tpsmat4s))]

            sql = "SELECT * FROM dmgs WHERE dmgs.idobject = '%d'" % objectId
            dmgresult = gdb.execute(sql)
            for dmgid, _, placementpose_str, _, planevector_str in dmgresult:
                placementid = self.getPlacementId(pg.mat4ToNp(dc.strToMat4(placementpose_str)))

                # search angle ranges
                sql = "SELECT idanglerange, grasporder FROM angleranges WHERE iddmg = %d" % dmgid
                anglerangesresult = gdb.execute(sql)
                anglerange_list = []
                anglerangeid_list = []
                for anglerange_id, grasporder_str in anglerangesresult:
                    anglerangeid_list.append(anglerange_id)
                    grasporder = [int(n) for n in grasporder_str[1:-1].split(',')]
                    newanglerange = []
                    for g in range(len(grasporder)):
                        sql = "SELECT pose FROM grasps WHERE idgrasp = %d" % grasporder[g]
                        graspresult = gdb.execute(sql)
                        newanglerange.append(pg.mat4ToNp(dc.strToMat4(graspresult[0][0])))
                    anglerange_list.append(newanglerange)

                # search for edges
                dmg_edges = []
                sql = "SELECT idanglerange1, idanglerange2, connectid1, connectid2 FROM dmgedges WHERE iddmg = %d" % dmgid
                dmgedgesresult = gdb.execute(sql)
                for anglerangeid1, anglerangeid2, connectid1, connectid2 in dmgedgesresult:
                    edge = [anglerangeid_list.index(anglerangeid1), anglerangeid_list.index(anglerangeid2), (connectid1, connectid2)]
                    dmg_edges.append(edge)

                self.regrasp_graph[placementid].append((np.array(dc.strToV6(planevector_str)), dexterousManipulationGraph(anglerange_list, dmg_edges, True)))

        else:
            print("the dmg table of ", self.dbobjname, " does not exist!!")

    def loadFreeTabletopPlacement(self):
        freetabletopplacementdata = self.gdb.loadFreeTabletopPlacementIncludeFF(self.dbobjname)
        if freetabletopplacementdata is None:
            raise ValueError("Plan the freeairgrip first!")
        self.tpsmat4s, self.placementid, self.placementtype = freetabletopplacementdata

    def loadFreeAirGripWithPlacementId(self, placementid):
        handrotmat = []
        hndjawwidth = []
        hndcontactpoint0 = []
        hndcontactpoint1 = []
        sql = "SELECT freetabletopgrip.rotmat, freetabletopgrip.jawwidth, freetabletopgrip.contactpoint0, freetabletopgrip.contactpoint1 FROM freetabletopgrip WHERE \
                freetabletopgrip.idfreetabletopplacement=%d" % placementid
        result = self.gdb.execute(sql)
        for resultrow in result:
            handrotmat.append(dc.strToMat4(resultrow[0]))
            hndjawwidth.append(float(resultrow[1]))
            hndcontactpoint0.append(dc.strToV3(resultrow[2]))
            hndcontactpoint1.append(dc.strToV3(resultrow[3]))

        return handrotmat, hndjawwidth, hndcontactpoint0, hndcontactpoint1

    def loadFreeAirGripForPlacement(self):
        self.gripsOfPlacement = []
        for i in range(len(self.placementid)):
            handrotmat, hndjawwidth, hndcontactpoint0, hndcontactpoint1 = self.loadFreeAirGripWithPlacementId(self.placementid[i])
            self.gripsOfPlacement.append((handrotmat, hndjawwidth, hndcontactpoint0, hndcontactpoint1))

    def getStep(self, start_grasp, goal_grasp):

        rotation = R.from_dcm(np.dot(np.transpose(start_grasp[:3,:3]), goal_grasp[:3,:3]))
        rotation_rotvec = rotation.as_rotvec()
        translation = goal_grasp[:3,3] - start_grasp[:3,3]
        numberOfStep = max(int(np.linalg.norm(rotation_rotvec) / 0.05) + int(np.linalg.norm(translation) / 5), 1)
        
        # get the rotation step
        rotationStep = R.from_rotvec( rotation_rotvec / numberOfStep).as_dcm()
        # get the translation step
        translationStep = translation / numberOfStep

        return rotationStep, translationStep, numberOfStep

    # this function will take two grasps and check whether one can move to another directly without collision.
    # render you need to render the object first for collision detection
    # start_grasp: SE(3)
    # goal_grasp: SE(3)
    def checkCollisionBetweenGrasps(self, startGrasp, goalGrasp, jawwidth, base):
        
        result = True
        # collisionId = 0
        trajectory = self.getLinearPoseTrajectory(startGrasp, goalGrasp)
        # self.hand.setJawwidth(jawwidth * 1000.0)
        self.hand.setJawwidth(jawwidth)
        self.hand.reparentTo(base.render)
        for i, t in enumerate(trajectory):
            self.hand.setMat(pandanpmat4 = pg.cvtMat4np4(t))
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(self.hand.handnp)
            result0 = self.bulletworldhp.contactTest(hndbullnode)
            result1 = self.bulletObj.contactTest(hndbullnode)

            cct0, cct1 = self.hand.getFingerTips()
            cct0 = self.getPointFromPose(pg.cvtMat4np4(t), cct0)
            cct1 = self.getPointFromPose(pg.cvtMat4np4(t), cct1)
            if result0.getNumContacts() or result1.getNumContacts() or (not self.bulletObj.rayTestAll(cct0, cct1).hasHits()):
                result = False
                break

        return result#, trajectory[collisionId]

    # this function will return the specific grasp g which belongs to placement i
    def getGrasp(self, i, g):
        return self.gripsOfPlacement[i][0][g], self.gripsOfPlacement[i][1][g]

    def getPlacement(self, i):
        return self.tpsmat4s[i]

    # this function will render the object with given placement which is panda form
    def renderObject(self, base, placement):

        if self.star != None:
            self.bulletObj.removeRigidBody(self.objectnp)
            self.star.removeNode()
            
        geom = pg.packpandageom(self.objtrimesh.vertices,
                                self.objtrimesh.face_normals,
                                self.objtrimesh.faces)

        node = GeomNode('obj')
        node.addGeom(geom)
        self.star = NodePath('obj')
        self.star.attachNewNode(node)
        self.star.setColor(Vec4(.7,0.3,0,1))
        self.star.setTransparency(TransparencyAttrib.MAlpha)
        self.star.setMat(placement)
        self.star.reparentTo(base.render)
        self.objectnp = cd.genCollisionMeshMultiNp(self.star)
        self.bulletObj.attachRigidBody(self.objectnp)

    def cleanRenderedObject(self, base):
        if self.star != None:
            self.bulletObj.removeRigidBody(self.objectnp)
            self.star.removeNode()
            self.star = None
    
    def getLinearPoseTrajectory(self, startPose, goalPose):
        """
        There is no collsion involve.
        """
        rotationStep, translationStep, numberOfStep = self.getStep(startPose, goalPose)

        poseTrajectory = []
        currentPose = np.copy(startPose)
        for _ in range(numberOfStep):
            poseTrajectory.append(currentPose.copy())
            currentPose[:3,:3] = currentPose[:3,:3].dot(rotationStep)
            currentPose[:3,3] = currentPose[:3,3] + translationStep
        poseTrajectory.append(goalPose.copy())
        return poseTrajectory

    # the plane is represneted by (x,y,z,nx,ny,nz) where nx,ny,nz is the normal vector of the plane
    def isSamePlane(self, plane1, plane2, threshold = 3.0):
        if plane1[3] * (plane2[0] - plane1[0]) + plane1[4] * (plane2[1] - plane1[1]) + plane1[5] * (plane2[2] - plane1[2]) < threshold \
            and plane2[3] * (plane1[0] - plane2[0]) + plane2[4] * (plane1[1] - plane2[1]) + plane2[5] * (plane1[2] - plane2[2]) < threshold \
            and np.linalg.norm(plane1[3:] - plane2[3:]) < 0.3:
            return True
        else:
            return False

    def group_grasp_points_by_planes(self, placementid):
        # find all point pairs
        point_pairs = []

        # get all contact point pair relative to current placement
        contactpoint0s = self.gripsOfPlacement[placementid][2]
        contactpoint1s = self.gripsOfPlacement[placementid][3]
        
        # ensure there are no duplicated contact point pairs
        for c0, c1 in zip(contactpoint0s, contactpoint1s):
            isNew = True
            for r in point_pairs:
                if np.linalg.norm(r[0] - c0) < 0.1 and np.linalg.norm(r[1] - c1) < 0.1:
                    isNew = False
                    break
            if isNew:
                point_pairs.append((np.array([c0[0], c0[1], c0[2]]),np.array([c1[0], c1[1], c1[2]])))


        point_pairs_lists_temp = []
        plane_of_point_pairs_list = []
        plane_lists_temp = []
        # group point pairs if they share the same plane
        for c0, c1 in point_pairs:
            middle_point = (c0 + c1) / 2.0
            middle_point_direction = (c1 - c0) / np.linalg.norm(c1 - c0)
            plane_normal = np.concatenate([middle_point, middle_point_direction])
            isNew = True
            for plane_index, p in enumerate(plane_lists_temp):
                if self.isSamePlane(p, plane_normal):
                    point_pairs_lists_temp[plane_index].append((c0, c1))
                    plane_of_point_pairs_list[plane_index].append(p)
                    isNew = False
                    break
            if isNew:
                plane_lists_temp.append(plane_normal)
                point_pairs_lists_temp.append([(c0,c1)])
                plane_of_point_pairs_list.append([plane_normal])

        plane_lists= []
        point_pairs_lists = []

        # recalculate the plane, ensure grasps stay into the same plane
        for i in range(len(plane_lists_temp)):
            if len(plane_of_point_pairs_list[i]) < 1:
                continue
            average_plane_normal = np.mean(np.array(plane_of_point_pairs_list[i]), axis=0)
            average_plane_normal[3:] = average_plane_normal[3:] / np.linalg.norm(average_plane_normal[3:])
            plane_lists.append(average_plane_normal)
            point_pairs_lists.append(point_pairs_lists_temp[i])

        return point_pairs_lists, plane_lists

    def getAngleBetweenDirection(self, v1, v2):
        uv1 = v1 / np.linalg.norm(v1)
        uv2 = v2 / np.linalg.norm(v2)
        tmp = np.dot(uv1, uv2)
        if 1.0 < tmp < 1.0 + 1e-6:
            tmp = 1.0
        elif -1.0 - 1e-6 < tmp < -1.0:
            tmp = -1.0
        return np.arccos(tmp)

    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])

    def generate_angle_and_feasibleMask(self, point_pairs, plane_normal, base):
        """
        For each point pair, it generate a set of grasp sharing this pair, and a mask bit which represents its valid.
        remind: each angle set, they could have multiple angle ranges.
        """
        discretesize = 16 # this value should not be 0 or 1
        point_pairs_temp = list(point_pairs)
        angleSet = []
        angleMask = []
        
        for c0, c1 in point_pairs_temp:
            if self.getAngleBetweenDirection(c1-c0, plane_normal) < (np.pi/2):
                tmp = c0
                c0 = c1
                c1 = tmp
            angleSet.append([])
            mask = [True] * discretesize
            for angleid in range(discretesize):
                tmphand = fetch_grippernm.Fetch_gripperNM(hndcolor=[1, 0, 0, .7])
                # initmat = tmphand.getMat()
                # save initial hand pose
                fgrv = c0 - c1
                fgrdist = np.linalg.norm(fgrv)
                tmphand.setJawwidth(fgrdist + 3)
                fgrv = fgrv / fgrdist
                tmphand.lookAt(fgrv[0], fgrv[1], fgrv[2])
                rotax = [0, 1, 0]
                rotangle = 360.0 / discretesize * angleid
                rotmat = rm.rodrigues(rotax, rotangle)
                anglegrasp = pandageom.cvtMat4(rotmat) * tmphand.getMat()
                tmphand.setMat(pandanpmat4= anglegrasp)
                axx = tmphand.getMat().getRow3(0)
                # 130 is the distance from hndbase to fingertip
                cctcenter = (c0 + c1) / 2 - tmphand.fingertipsOffset * np.array([axx[0], axx[1], axx[2]])
                tmphand.setPos(npvec3=Point3(cctcenter[0], cctcenter[1], cctcenter[2]))
                angleSet[-1].append(tmphand.getMat())

                # collision detection
                hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)

                result0 = self.bulletworldhp.contactTest(hndbullnode)
                result1 = self.bulletObj.contactTest(hndbullnode)
                if result0.getNumContacts() or result1.getNumContacts():
                    mask[angleid] = False

                tmphand.removeNode()

            # split into angleranges
            startrange = False
            newangleRange = [False] * discretesize

            result = []

            for m in range(len(mask)):
                
                if mask[m]:
                    if not startrange:
                        newangleRange = [False] * discretesize
                        startrange = True
                    newangleRange[m] = True
                else:
                    if startrange:
                        result.append(newangleRange)
                        startrange = False

                if m == len(mask) - 1 and mask[m]:
                    if mask[0]:
                        for r in range(discretesize):
                            if newangleRange[r]:
                                result[0][r] = True
                    else:
                        result.append(newangleRange)

            angleMask.append(result)

        return angleSet, angleMask

    def get_contact_point_edge_by_voronoi(self, grasp_point_pairs, plane):

        rotateMatrix = trigeom.align_vectors(plane[3:], [0,0,1])
        # project 3d point to 2d plane
        grasp_points_2d = []
        for i in range(len(grasp_point_pairs)):
            grasp_points_2d.append(rm.transformmat4(rotateMatrix, grasp_point_pairs[i][0])[:2])
        grasp_points_2d = np.array(grasp_points_2d)

        if grasp_points_2d.shape[0] > 2:
            return Voronoi(grasp_points_2d).ridge_points
        elif grasp_points_2d.shape[0] == 2:
            return np.array([[0, 1]])
        elif grasp_points_2d.shape[0] == 1:
            return np.array([[0, 0]])

    def getCommonBit(self, mask1, mask2):
        result = False
        firstOfMask1 = 0
        firstOfMask2 = 0
        firstOfAngleRange1 = None
        firstOfAngleRange2 = None
        commonBit = [False] * len(mask1)
        # calculate the both side of each mask
        for i in range(len(mask1)):
            if mask1[i] and mask2[i]:
                result = True
                commonBit[i] = True
        firstOfAngleRange1, _ = self.getSideOfAngleRange(mask1)
        firstOfAngleRange2, _ = self.getSideOfAngleRange(mask2)
        first, end = self.getSideOfAngleRange(commonBit)

        if first == None and end == None:
            return None, None
        mvalue = None
        if end >= first:
            mvalue = int((end + first)/2)
        else:
            mvalue = int((first + len(commonBit) + 1)/2)
            if mvalue >= len(commonBit):
                mvalue -= len(commonBit)

        if firstOfAngleRange1 <= mvalue:
            firstOfMask1 = mvalue - firstOfAngleRange1
        else:
            firstOfMask1 = mvalue + len(commonBit) - firstOfAngleRange1

        if firstOfAngleRange2 <= mvalue:
            firstOfMask2 = mvalue - firstOfAngleRange2
        else:
            firstOfMask2 = mvalue + len(commonBit) - firstOfAngleRange2

        return result, (firstOfMask1, firstOfMask2)

    def rotate(self, angleRange, n):
        """
        rotate the list with n step
        """
        return angleRange[n:] + angleRange[:n]

    def getSideOfAngleRange(self, mask):
        firstOfAngleRange = None
        endOfAngleRange = None
        isloop = False
        for i in range(len(mask)):
            if mask[i]:
                if i == len(mask) - 1 and endOfAngleRange == None:
                    endOfAngleRange = i
                    if firstOfAngleRange == None:
                        firstOfAngleRange = i
                elif endOfAngleRange != None and not isloop:
                    firstOfAngleRange = i
                    isloop = True
                elif firstOfAngleRange == None:
                    firstOfAngleRange = i
            elif firstOfAngleRange != None and endOfAngleRange == None:
                endOfAngleRange = i - 1
        return firstOfAngleRange, endOfAngleRange

    def build_regrasp_graph(self, plane, point_pairs, base):
        """
        Given a placement and one of its motion plane and relative contact point pairs, 
        this function will generate a dmg which is used to generate the end-effector motion
        over the object surface.
        """

        # for each point pair, we generate its angle set and valid mask.
        angleSet, angleMask = self.generate_angle_and_feasibleMask(point_pairs, plane[3:], base)

        # according to the plane and point pairs, we generate a voroni graph and connect them.
        voroni_grasp_point_edges = self.get_contact_point_edge_by_voronoi(point_pairs, plane)
        angleRange_edges = []
        for idx1, idx2 in voroni_grasp_point_edges:
            for i in range(len(angleMask[idx1])):
                for j in range(len(angleMask[idx2])):
                    isConnect, commonBit = self.getCommonBit(angleMask[idx1][i], angleMask[idx2][j]) # we may also store the common edges grasps

                    if isConnect:
                        firstlist = [a for a, b in enumerate(angleMask[idx1][i]) if b]
                        secondlist = [a for a, b in enumerate(angleMask[idx2][j]) if b]
                        if self.checkCollisionBetweenGrasps(startGrasp=pg.mat4ToNp(angleSet[idx1][firstlist[commonBit[0]]]), goalGrasp=pg.mat4ToNp(angleSet[idx2][secondlist[commonBit[1]]]), jawwidth=100, base=base):
                            # firstpoint = self.getPointFromPose(angleSet[idx1][firstlist[commonBit[0]]], Point3(self.hand.contactPointOffset, 0, 0))
                            # secondpoint = self.getPointFromPose(angleSet[idx2][secondlist[commonBit[1]]], Point3(self.hand.contactPointOffset, 0, 0))
                            # pandageom.plotLinesegs(base.render, np.array([[firstpoint[0], firstpoint[1], firstpoint[2]],[secondpoint[0], secondpoint[1], secondpoint[2]]]))
                            angleRange_edges.append(((idx1, i),(idx2,j),commonBit))

        angleRange_list = [[] for _ in range(len(angleMask))]

        # generate angle ranges
        # with following code, each side of the angle range will be at the beginning or ending of the angle range list.
        for i in range(len(angleMask)):
            for j in range(len(angleMask[i])):
                firstOfAngleRange, endOfAngleRange = self.getSideOfAngleRange(angleMask[i][j])
                newAngleRange = [pg.mat4ToNp(grasp) for indx, grasp in enumerate(angleSet[i]) if angleMask[i][j][indx]]

                if endOfAngleRange < firstOfAngleRange:
                    newAngleRange = self.rotate(newAngleRange, endOfAngleRange + 1)

                angleRange_list[i].append(newAngleRange)

        # build the DMG with angleRange and angleRange_edges
        DMG = dexterousManipulationGraph(angleRange_list, angleRange_edges)
        return DMG

    def build_regrasp_graph_for_all_placements(self, base):
        """
        generate dmg for each placement
        For each element of regrasp_graph, it contains a list of pair with plane and its dmg.
        """
        # print("number of placement = ", len(self.tpsmat4s))
        self.regrasp_graph = [[] for _ in range(len(self.tpsmat4s))]
        for placementid, placement in enumerate(tqdm(self.tpsmat4s)):


            # need to group grasp point pairs by plane in one placement
            # i-th list of point pairs list is all contact point pair in the i-th motion plane.
            point_pairs_list, plane_list = self.group_grasp_points_by_planes(placementid)

            # need to render the object for current placement
            self.renderObject(base, placement)
            
            for plane, point_pairs in zip(plane_list, point_pairs_list):
                self.regrasp_graph[placementid].append((plane, self.build_regrasp_graph(plane, point_pairs, base)))

            self.cleanRenderedObject(base)

    def getPointFromPose(self, pose, point):
        return Point3(pose[0][0] * point[0] + pose[1][0] * point[1] + pose[2][0] * point[2] + pose[3][0], \
                      pose[0][1] * point[0] + pose[1][1] * point[1] + pose[2][1] * point[2] + pose[3][1], \
                      pose[0][2] * point[0] + pose[1][2] * point[1] + pose[2][2] * point[2] + pose[3][2])

    def getPlacementId(self, placement):
        # get the proper placement id
        currentPlacementDirection = pg.getGroundDirection(placement)
        placementDirections = [pg.getGroundDirection(pg.mat4ToNp(g)) for g in self.tpsmat4s]

        diff = 2.0 
        closest_placementid = None
        for p in range(len(placementDirections)):
            pdirection = placementDirections[p]
            
            currnt_diff = np.linalg.norm(pdirection - currentPlacementDirection)
            if currnt_diff <= diff:
                closest_placementid = p
                diff = currnt_diff

        return closest_placementid

    def getDMGWithPlacementAndGrasp(self, placement_id, init_grasp, target_grasp):
        

        # get finger direction of init grasp
        tmphand = fetch_grippernm.Fetch_gripperNM(hndcolor=[1, 0, 0, 0])
        tmphand.setMat(pandanpmat4= pg.cvtMat4np4(init_grasp))
        tmphand.setJawwidth(50)
        c1, c0 = tmphand.getFingerTips()
        c0 = self.getPointFromPose(pg.cvtMat4np4(init_grasp), c0)
        c1 = self.getPointFromPose(pg.cvtMat4np4(init_grasp), c1)
        init_grasp_direction = np.array([c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2]])
        init_grasp_direction = init_grasp_direction / np.linalg.norm(init_grasp_direction)
        init_grasp_center = np.array([c1[0] + c0[0], c1[1] + c0[1], c1[2] + c0[2]]) / 2
        init_grasp_plane = np.concatenate((init_grasp_center, init_grasp_direction))

        # get finger firection of target grasp
        tmphand.setMat(pandanpmat4= pg.cvtMat4np4(target_grasp))
        tmphand.setJawwidth(50)
        c1, c0 = tmphand.getFingerTips()
        c0 = self.getPointFromPose(pg.cvtMat4np4(target_grasp), c0)
        c1 = self.getPointFromPose(pg.cvtMat4np4(target_grasp), c1)
        target_grasp_direction = np.array([c1[0] - c0[0], c1[1] - c0[1], c1[2] - c0[2]])
        target_grasp_direction = target_grasp_direction / np.linalg.norm(target_grasp_direction)
        target_grasp_center = np.array([c1[0] + c0[0], c1[1] + c0[1], c1[2] + c0[2]]) / 2
        target_grasp_plane = np.concatenate((target_grasp_center, target_grasp_direction))

        init_grasp_plane_id = None
        target_grasp_plane_id = None

        for i in range(len(self.regrasp_graph[placement_id])):
            plane_direction = self.regrasp_graph[placement_id][i][0]
            if self.isSamePlane(init_grasp_plane, plane_direction, 15.0):
                init_grasp_plane_id = i
            if self.isSamePlane(target_grasp_plane, plane_direction, 15.0):
                target_grasp_plane_id = i

        tmphand.removeNode()

        return init_grasp_plane_id, target_grasp_plane_id


    # need to render the object before this function for collision detection
    # this function will return the angleRange id and matching grasp in that angle range
    def get_closest_angle_range(self, angleRanges, grasp, jawwidth, base):
        "Given the a grasp, this function will return the closeset angel range to that grasp"

        grasp_inv_rot = np.transpose(grasp[:3,:3])
        grasp_trans = grasp[:,3:]

        pose_angleRange = []
        rot_diff = []
        tran_diff = []

        for a, angel_range in enumerate(angleRanges):
            for e, pose in enumerate(angel_range):
                pos_rot = pose[:3,:3]
                pos_tran = pose[:,3:]

                rot = R.from_dcm(np.dot(grasp_inv_rot,pos_rot))
                crnt_rot_diff = np.linalg.norm(rot.as_rotvec())
                crnt_tran_diff = np.linalg.norm(pos_tran - grasp_trans)
                rot_diff.append(crnt_rot_diff)
                tran_diff.append(crnt_tran_diff)
                pose_angleRange.append([pose, a, 0, e])

        tran_diff = softmax(tran_diff)
        rot_diff = softmax(rot_diff)

        for i in range(len(pose_angleRange)):
            pose_angleRange[i][2] = tran_diff[i] + rot_diff[i]

        def sortfun(e):
            return e[2]
        pose_angleRange.sort(key=sortfun) # pose angle range: (grasp pose, angle range id, pose difference)

        for p in pose_angleRange:
            if self.checkCollisionBetweenGrasps(grasp, p[0], jawwidth, base):
                return p[1], p[3]

        return None, None

    # This function will read numpy pose matrix
    def getTrajectory(self, _init_grasp, _target_grasp, jawwidth, _placement, base):

        # get the matching object placement pose
        placementid = self.getPlacementId(_placement)
        placement = pg.mat4ToNp(self.tpsmat4s[placementid])

        # get the init grasp and target grasp in table frame
        init_grasp = placement.dot(_init_grasp)
        target_grasp = placement.dot(_target_grasp)
        
        init_grasp_plane_id, target_grasp_plane_id = self.getDMGWithPlacementAndGrasp(placementid, init_grasp, target_grasp)

        if init_grasp_plane_id == None or target_grasp_plane_id == None:
            print("init grasp or target grasp does not belong to any plane")
            return None
        if init_grasp_plane_id != target_grasp_plane_id:
            print("init grasp and target grasp does not share the same plane")
            return None

        self.renderObject(base, self.tpsmat4s[placementid])

        dmg = self.regrasp_graph[placementid][init_grasp_plane_id][1]
        angleRanges = dmg.getAngleRange()

        angleRange_idx_init_grasp, init_connect_grasp = self.get_closest_angle_range(angleRanges, init_grasp, jawwidth, base)
        angleRange_idx_target_grasp, target_connect_grasp = self.get_closest_angle_range(angleRanges, target_grasp, jawwidth, base)

        if angleRange_idx_init_grasp == None:
            self.cleanRenderedObject(base)
            print("can't find angle range index of initial grasp")
            return None
        if angleRange_idx_target_grasp == None:
            self.cleanRenderedObject(base)
            print("can't find angle range index of target grasps")
            return None

        dmg.insert_init_grasp_2_DMG(angleRange_idx_init_grasp, init_grasp, init_connect_grasp)
        dmg.insert_end_grasp_2_DMG(angleRange_idx_target_grasp, target_grasp, target_connect_grasp)
        poseTrajectory = dmg.getGraspTrajectory()
        dmg.remove_init_and_end_grasp_from_DMG()

        if poseTrajectory == None:
            self.cleanRenderedObject(base)
            print("can't find pose trajectory for unknown reason")
            return None

        # need to shorten the trajectory
        shortenPoseTrajectory = []
        i = 0
        j = len(poseTrajectory) - 1
        shortenPoseTrajectory.append(poseTrajectory[i])
        while i < len(poseTrajectory) - 1:
            while i < j:
                if i + 1 == j or self.checkCollisionBetweenGrasps(poseTrajectory[i], poseTrajectory[j], jawwidth, base):
                    shortenPoseTrajectory.append(poseTrajectory[j])
                    i = j
                    j = len(poseTrajectory) - 1
                else:
                    j -= 1

        self.cleanRenderedObject(base)

        # convert the pose Trajectory from table frame back to object frame
        result = []
        inv_placement_mat = np.linalg.inv(placement)
        for pose in shortenPoseTrajectory:
          result.append(inv_placement_mat.dot(pose))
        return result

# if __name__ == '__main__':

#     base = pandactrl.World(camp=[700,300,1400], lookatp=[0,0,0])
#     this_dir, this_filename = os.path.split(__file__)
#     objpath = os.path.join(this_dir, "objects", "book.stl")

#     handpkg = fetch_grippernm
#     gdb = db.GraspDB()
#     regrasp_planner = ff_regrasp_planner(objpath, handpkg, gdb)
#     regrasp_planner.build_regrasp_graph_for_all_placements(base)

#     placementId = 3
#     startGraspId = 1
#     goalGraspId = 10

#     startPose_panda, startJawwidth_panda = regrasp_planner.getGrasp(placementId, startGraspId)
#     goalPose_panda, goalJawwidth_panda = regrasp_planner.getGrasp(placementId, goalGraspId)
#     placementPose = regrasp_planner.getPlacement(placementId)
#     inv_placementPose = np.linalg.inv(pg.mat4ToNp(placementPose))
#     # pass the init grasp and target grasp in object frame
#     # pass the object placement in the table frame
#     grasp_trajectory = regrasp_planner.getTrajectory(inv_placementPose.dot(pg.mat4ToNp(startPose_panda)), inv_placementPose.dot(pg.mat4ToNp(goalPose_panda)), 0.08, pg.mat4ToNp(placementPose), base)


#     poseTrajectory = []
#     for g in range(len(grasp_trajectory) - 1):
#         trajectory = regrasp_planner.getLinearPoseTrajectory(pg.mat4ToNp(placementPose).dot(grasp_trajectory[g]), pg.mat4ToNp(placementPose).dot(grasp_trajectory[g+1]))
#         poseTrajectory.extend(trajectory)

#     ################ following code is used to demo ####################################################
#     # poseTrajectory: a grasp pose trajectory from start pose to goal pose
#     # create a hand for demo
#     # starthnd = regrasp.handpkg.newHandNM(hndcolor=[0, 0, 1, 0.5])
#     # starthnd.setMat(pandanpmat4 = startPose_panda)
#     # starthnd.setJawwidth(startJawwidth_panda)
#     # starthnd.reparentTo(base.render)

#     # goalhnd = regrasp.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
#     # goalhnd.setMat(pandanpmat4 = goalPose_panda)
#     # goalhnd.setJawwidth(goalJawwidth_panda)
#     # goalhnd.reparentTo(base.render)

#     objtrimesh=trimesh.load_mesh(objpath)
#     geom = pg.packpandageom(objtrimesh.vertices,
#                             objtrimesh.face_normals,
#                             objtrimesh.faces)

#     node = GeomNode('obj')
#     node.addGeom(geom)
#     star = NodePath('obj')
#     star.attachNewNode(node)
#     star.setColor(Vec4(.7,0.3,0,1))
#     star.setTransparency(TransparencyAttrib.MAlpha)
#     star.setMat(placementPose)
#     star.reparentTo(base.render)
    
#     currenthnd = regrasp_planner.handpkg.newHandNM(hndcolor=[0, 0.5, 0.5, 0.5])
#     currenthnd.setJawwidth(startJawwidth_panda)
#     currenthnd.reparentTo(base.render)

#     counter = 0
#     def myFunction(task, poseTrajectory):
#         global counter
#         if counter > len(poseTrajectory) - 1:
#             return task.done

#         currenthnd.setMat(pandanpmat4 = pg.cvtMat4np4(poseTrajectory[counter]))
#         counter += 1
#         return task.again

#     # show the collision net
#     def updateworld(world, task):
#         world.doPhysics(globalClock.getDt())
#         return task.cont

#     debugNode = BulletDebugNode('Debug')
#     debugNode.showWireframe(True)
#     debugNode.showConstraints(True)
#     debugNode.showBoundingBoxes(False)
#     debugNode.showNormals(False)
#     bullcldrnp = base.render.attachNewNode("bulletcollider")
#     debugNP = bullcldrnp.attachNewNode(debugNode)
#     debugNP.show()
#     regrasp_planner.bulletworldhp.setDebugNode(debugNP.node())
#     taskMgr.add(updateworld, "updateworld", extraArgs=[regrasp_planner.bulletworldhp], appendTask=True)

#     myTask = taskMgr.doMethodLater(0.1, myFunction, 'tickTask', extraArgs=[Task.Task(myFunction), poseTrajectory])

#     base.run()