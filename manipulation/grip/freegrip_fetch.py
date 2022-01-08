#!/usr/bin/python
import os

import MySQLdb as mdb
import numpy as np

from manipulation.grip.fetch_gripper import fetch_grippernm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from manipulation.grip import freegripcontactpairs as fgcp
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from database import dbaccess as db
from panda3d.bullet import BulletDebugNode

class Freegrip(fgcp.FreegripContactpairs):

    def __init__(self, objpath, handpkg, readser=False, torqueresist = 50):
        """
        initialization

        :param objpath: path of the object
        :param ser: True use pre-computed template file for debug (in order to debug large models like tool.stl
        :param torqueresist: the maximum allowable distance to com (see FreegripContactpairs.planContactpairs)
        """

        super(self.__class__, self).__init__(ompath=objpath, numberOfSamplingPoints=100, readser=readser)
        if readser is False:
            # use this one to set the max and min distance from fingertips to the boundary
            # self.removeBadSamples(mindist=1, maxdist=25)
            # self.clusterFacetSamplesRNN(reduceRadius=3)
            self.removeBadSamples(mindist=8, maxdist=25)
            self.clusterFacetSamplesRNN(reduceRadius=8)
            self.planContactpairs(torqueresist, fgrtipdist = 30)
            # self.saveSerialized("tmpcp.pickle")
        else:
            pass
            # self.loadSerialized("tmpcp.pickle", objpath)

        self.handpkg = handpkg
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.1])
        self.handfgrpcc_uninstanced = handpkg.newHandFgrpcc()
        self.handname = handpkg.getHandName()

        # gripcontactpairs_precc is the gripcontactpairs ([[p0,p1,p2],[p0',p1',p2']] pairs) after precc (collision free)
        # gripcontactpairnormals_precc is the gripcontactpairnormals ([[n0,n1,n2],[n0',n1',n2']] pairs) after precc
        # likewise, gripcontactpairfacets_precc is the [faceid0, faceid1] pair corresponding to the upper two
        self.gripcontactpairs_precc = None
        self.gripcontactpairnormals_precc = None
        self.gripcontactpairfacets_precc = None

        # the final results: gripcontacts: a list of [cct0, cct1]
        # griprotmats: a list of Mat4
        # gripcontactnormals: a list of [nrml0, nrml1]
        self.gripcontacts = None
        self.griprotmats = None
        self.gripjawwidth = None
        self.gripcontactnormals = None

        self.bulletworld = BulletWorld()
        # prepare the model for collision detection
        self.objgeom = pandageom.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.objmeshbullnode = cd.genCollisionMeshGeom(self.objgeom)
        self.bulletworld.attachRigidBody(self.objmeshbullnode)

        # for dbupdate
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]
        print("db object name ", self.dbobjname)


    def removeHndcc(self, base, discretesize=8):
        """
        Handcc means hand collision detection

        :param discretesize: the number of hand orientations
        :return:
        """
        print("remove hand collision check")


        self.gripcontacts = []
        self.griprotmats = []
        self.gripjawwidth = []
        self.gripcontactnormals = []

        plotoffsetfp = 5

        self.counter = 0

        while self.counter < self.facetpairs.shape[0]:
            print(str(self.counter + 1) + "/" + str(self.facetpairs.shape[0]))

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            tmphand = fetch_grippernm.Fetch_gripperNM(hndcolor=[1, 0, 0, .1])

            for j, contactpair in enumerate(self.gripcontactpairs_precc[self.counter]):
                for angleid in range(discretesize):
                    cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                    cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                    cctnormal0 = self.gripcontactpairnormals_precc[self.counter][j][0]

                    
                    # save initial hand pose
                    initmat = tmphand.getMat()
                    fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))
                    tmphand.setJawwidth(fgrdist)
                    tmphand.lookAt(cctnormal0[0], cctnormal0[1], cctnormal0[2])
                    rotax = [0, 1, 0]
                    rotangle = 360.0 / discretesize * angleid
                    rotmat = rm.rodrigues(rotax, rotangle)
                    tmphand.setMat(pandanpmat4=pandageom.cvtMat4(rotmat) * tmphand.getMat())
                    axx = tmphand.getMat().getRow3(0)
                    # 130 is the distance from hndbase to fingertip
                    cctcenter = (cctpnt0 + cctpnt1) / 2 - tmphand.fingertipsOffset * np.array([axx[0], axx[1], axx[2]])
                    tmphand.setPos(npvec3=Point3(cctcenter[0], cctcenter[1], cctcenter[2]))

                    # collision detection
                    hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)
                    result = self.bulletworld.contactTest(hndbullnode)

                    if not result.getNumContacts():
                        self.gripcontacts.append(contactpair)
                        self.griprotmats.append(tmphand.getMat())
                        self.gripjawwidth.append(fgrdist)
                        self.gripcontactnormals.append(self.gripcontactpairnormals_precc[self.counter][j])

                        # need to add a flipped one too
                        flippedcontactpair = [contactpair[1], contactpair[0]]
                        flippedcontactnormals = [self.gripcontactpairnormals_precc[self.counter][j][1], 
                                                 self.gripcontactpairnormals_precc[self.counter][j][0]]
                        self.gripcontacts.append(flippedcontactpair)
                        self.griprotmats.append(pandageom.cvtMat4(rm.rodrigues([1, 0, 0], 180)) * tmphand.getMat())
                        self.gripjawwidth.append(fgrdist)
                        self.gripcontactnormals.append(flippedcontactnormals)

                    # reset initial hand pose
                    tmphand.setMat(initmat)

            self.counter+=1

        self.counter = 0

    def removeFgrpcc(self, base):
        """
        Fgrpcc means finger pre collision detection

        :return:
        """
        print("remove finger collision check")

        self.gripcontactpairs_precc = []
        self.gripcontactpairnormals_precc = []
        self.gripcontactpairfacets_precc = []

        plotoffsetfp = 5

        self.counter = 0

        while self.counter < self.facetpairs.shape[0]:
            print(str(self.counter + 1) + "/" + str(self.facetpairs.shape[0]))
            self.gripcontactpairs_precc.append([])
            self.gripcontactpairnormals_precc.append([])
            self.gripcontactpairfacets_precc.append([])

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal0 = self.facetnormals[facetidx0]
                cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
                handfgrpcc0 = NodePath("handfgrpcc0")
                self.handfgrpcc_uninstanced.instanceTo(handfgrpcc0)
                handfgrpcc0.setPos(cctpnt0[0], cctpnt0[1], cctpnt0[2])
                handfgrpcc0.lookAt(cctpnt0[0] + cctnormal0[0], cctpnt0[1] + cctnormal0[1],
                                 cctpnt0[2] + cctnormal0[2])
                handfgrpcc1 = NodePath("handfgrpcc1")
                self.handfgrpcc_uninstanced.instanceTo(handfgrpcc1)
                handfgrpcc1.setPos(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                handfgrpcc1.lookAt(cctpnt1[0] + cctnormal1[0], cctpnt1[1] + cctnormal1[1],
                                 cctpnt1[2] + cctnormal1[2])
                handfgrpcc = NodePath("handfgrpcc")
                handfgrpcc0.reparentTo(handfgrpcc)
                handfgrpcc1.reparentTo(handfgrpcc)

                # prepare the model for collision detection
                facetmeshbullnode = cd.genCollisionMeshMultiNp(handfgrpcc)
                result = self.bulletworld.contactTest(facetmeshbullnode)

                # show the finger pcc
                # handfgrpcc.reparentTo(base.render)

                if not result.getNumContacts():
                    self.gripcontactpairs_precc[-1].append(contactpair)
                    self.gripcontactpairnormals_precc[-1].append(self.gripcontactpairnormals[self.counter][j])
                    self.gripcontactpairfacets_precc[-1].append(self.gripcontactpairfacets[self.counter])
            self.counter += 1
        self.counter=0

    def saveToDB(self, gdb):
        """
        save the result to mysqldatabase

        :param gdb: is an object of the GraspDB class in the database package
        :return:
        """

        # save to database
        gdb = db.GraspDB()

        idhand = gdb.loadIdHand(self.handname)

        sql = "SELECT * FROM freeairgrip, object WHERE freeairgrip.idobject = object.idobject AND \
                object.name LIKE '%s' AND freeairgrip.idhand LIKE '%s'" % (self.dbobjname, idhand)
        result = gdb.execute(sql)
        if not result:
            sql = "SELECT idobject FROM object WHERE name LIKE '%s'" % self.dbobjname
            returnlist = gdb.execute(sql)
            if len(returnlist) != 0:
                idobject = returnlist[0][0]
            else:
                sql = "INSERT INTO object(name) VALUES('%s')" % self.dbobjname
                idobject = gdb.execute(sql)
            for i in range(len(self.gripcontacts)):
                sql = "INSERT INTO freeairgrip(idobject, contactpnt0, contactpnt1, \
                        contactnormal0, contactnormal1, rotmat, jawwidth, idhand) \
                       VALUES('%s', '%s', '%s', '%s', '%s', '%s', '%s', %d)" % \
                      (idobject, dc.v3ToStr(self.gripcontacts[i][0]), dc.v3ToStr(self.gripcontacts[i][1]),
                       dc.v3ToStr(self.gripcontactnormals[i][0]), dc.v3ToStr(self.gripcontactnormals[i][1]),
                       dc.mat4ToStr(self.griprotmats[i]), str(self.gripjawwidth[i]), idhand)
                gdb.execute(sql)
        else:
            print("Grasps already saved or duplicated filename!")

    def plotObj(self):
        geomnodeobj = GeomNode('obj')
        geomnodeobj.addGeom(self.objgeom)
        npnodeobj = NodePath('obj')
        npnodeobj.attachNewNode(geomnodeobj)
        npnodeobj.reparentTo(base.render)

    def showAllGrips(self):
        """
        showAllGrips

        :return:
        """

        for i in range(len(self.gripcontacts)):
            # i = 2
            contactpair = self.gripcontacts[i]
            pandageom.plotSphere(base.render, pos=contactpair[0], radius=3, rgba=Vec4(1,0,0,1))
            pandageom.plotSphere(base.render, pos=contactpair[1], radius=3, rgba=Vec4(1,0,0,1))

            # if i == 6:
            hndrotmat = self.griprotmats[i]
            hndjawwidth = self.gripjawwidth[i]
            # show grasps
            tmpfetch = fetch_grippernm.Fetch_gripperNM(hndcolor=[0, 0, 1, .5])
            tmpfetch.setMat(pandanpmat4=hndrotmat)
            tmpfetch.setJawwidth(hndjawwidth)
            # tmpfetch.setJawwidth(80)
            tmpfetch.reparentTo(base.render)

            # pandageom.plotAxisSelf(base.render, spos=Vec3(hndrotmat.getCell(3,0),hndrotmat.getCell(3,1),hndrotmat.getCell(3,2)), pandamat4=hndrotmat)


if __name__=='__main__':

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)

    # objpath = os.path.join(this_dir, "objects", "cup.stl")
    # objpath = os.path.join(this_dir, "objects", "cuboid.stl")
    # objpath = os.path.join(this_dir, "objects", "book.stl")
    # objpath = os.path.join(this_dir, "objects", "box.stl")
    # objpath = os.path.join(this_dir, "objects", "cylinder.stl")
    # objpath = os.path.join(this_dir, "objects", "almonds_can.stl")
    objpath = os.path.join(this_dir, "objects", "Lshape.stl")
    # objpath = os.path.join(this_dir, "objects", "bottle.stl")

    handpkg = fetch_grippernm
    freegriptst = Freegrip(objpath, handpkg, readser=False, torqueresist = 120)

    freegriptst.removeFgrpcc(base)
    freegriptst.removeHndcc(base)

    gdb = db.GraspDB()
    freegriptst.saveToDB(gdb)

    # axis = loader.loadModel('zup-axis.egg')
    # axis.setScale(10)
    # axis.reparentTo(base.render)

    # def updateshow(task):
    #     # freegriptst.removeFgrpccShow(base)
    #     # freegriptst.removeFgrpccShowLeft(base)
    #     freegriptst.removeHndccShow(base)
    # #     # print(task.delayTime)
    # #     # if abs(task.delayTime-13) < 1:
    # #     #     task.delayTime -= 12.85
    #     return task.again
    #
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")
    # taskMgr.add(updateshow, "tickTask")
    # freegriptst.removeFgrpcc(base)
    # freegriptst.removeHndcc(base)

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont
    
    # base.taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)
    
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNode.showConstraints(True)
    # debugNode.showBoundingBoxes(False)
    # debugNode.showNormals(False)
    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    # freegriptst.bulletworld.setDebugNode(debugNP.node())
    # taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)

    freegriptst.plotObj()
    freegriptst.showAllGrips()

    base.run()
