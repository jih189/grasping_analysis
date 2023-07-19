#!/usr/bin/python
import os
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
from panda3d.bullet import BulletDebugNode

class Freegrip(fgcp.FreegripContactpairs):

    def __init__(self, objpath, handpkg, torqueresist = 200):
        """
        initialization

        :param objpath: path of the object
        :param ser: True use pre-computed template file for debug (in order to debug large models like tool.stl
        :param torqueresist: the maximum allowable distance to com (see FreegripContactpairs.planContactpairs)
        """
        super(self.__class__, self).__init__(ompath=objpath, numberOfSamplingPoints=50, readser=False, object_scale = 1000.0)

        # use this one to set the max and min distance from fingertips to the boundary
        self.removeBadSamples(mindist=3, maxdist=10)
        self.clusterFacetSamplesRNN(reduceRadius=7)
        # self.removeBadSamples(mindist=7, maxdist=25)
        # self.clusterFacetSamplesRNN(reduceRadius=8)
        self.planContactpairs(torqueresist, fgrtipdist = 80)
        # self.saveSerialized("tmpcp.pickle")

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

    def plotObj(self):

        pandageom.plotSphere(base.render, pos=Point3(0,0,0), radius=3, rgba=Vec4(1,0,0,1))

        geomnodeobj = GeomNode('obj')
        geomnodeobj.addGeom(self.objgeom)
        npnodeobj = NodePath('obj')
        npnodeobj.attachNewNode(geomnodeobj)
        npnodeobj.setColor(Vec4(.8,0.8,0.8,0.4))
        npnodeobj.setTransparency(TransparencyAttrib.MAlpha)
        npnodeobj.reparentTo(base.render)

    def saveGraspPoses(self, object_name):
        list_of_arrays = []
        for i in range(len(self.gripcontacts)):
            # convert it to numpy format

            hndrotmat = self.griprotmats[i]
            row0 = hndrotmat.getRow(0)
            row1 = hndrotmat.getRow(1)
            row2 = hndrotmat.getRow(2)
            row3 = hndrotmat.getRow(3)


            numpy_format = np.array([[row0[0], row1[0], row2[0], row3[0]], [row0[1], row1[1], row2[1], row3[1]],
                            [row0[2], row1[2], row2[2], row3[2]], [row0[3], row1[3], row2[3], row3[3]]])
            # scale it back to normal size
            numpy_format[:3, 3] *= 0.001
            list_of_arrays.append(numpy_format)

        np.savez(object_name + '.npz', *list_of_arrays)

    def showAllGrips(self):
        """
        showAllGrips

        :return:
        """

        print("number of grasp: ", len(self.gripcontacts))
        for i in range(len(self.gripcontacts)):
            contactpair = self.gripcontacts[i]
            pandageom.plotSphere(base.render, pos=contactpair[0], radius=3, rgba=Vec4(1,0,0,1))
            pandageom.plotSphere(base.render, pos=contactpair[1], radius=3, rgba=Vec4(1,0,0,1))

            hndrotmat = self.griprotmats[i]
            hndjawwidth = self.gripjawwidth[i]

            # show grasps
            tmpfetch = fetch_grippernm.Fetch_gripperNM(hndcolor=[0, 0, 1, .5])
            tmpfetch.setMat(pandanpmat4=hndrotmat)
            tmpfetch.setJawwidth(hndjawwidth)
            tmpfetch.reparentTo(base.render)
            # pandageom.plotAxisSelf(base.render, spos=Vec3(hndrotmat.getCell(3,0),hndrotmat.getCell(3,1),hndrotmat.getCell(3,2)), pandamat4=hndrotmat)


if __name__=='__main__':

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)

    objpath = os.path.join(this_dir, "objects", "cup.stl")

    object_name = os.path.splitext(os.path.basename(objpath))[0]

    freegriptst = Freegrip(objpath, fetch_grippernm, torqueresist = 500)

    freegriptst.removeFgrpcc(base)
    freegriptst.removeHndcc(base)

    freegriptst.saveGraspPoses(object_name)

    # freegriptst.plotObj()
    # freegriptst.showAllGrips()

    # base.run()
