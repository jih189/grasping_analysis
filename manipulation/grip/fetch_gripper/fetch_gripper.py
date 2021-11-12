# TODO: reduce the dependency on panda3d

import math
import os

import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *

from utils import designpattern

class Fetch_gripper():
    '''
    use utils.designpattern.singleton() to get a single instance of this class
    '''

    def __init__(self, jawwidth=0.0):
        '''
        load the fetch gripper model, set jawwidth and return a nodepath
        the Fetch gripper is composed of a parallelism and a fixed triangle,
        the parallelism: 1.905-1.905; 5.715-5.715; 70/110 degree
        the triangle: 4.75 (finger) 5.715 (inner knuckle) 3.175 (outer knuckle)

        ## input
        pandabase:
            the showbase() object
        jawwidth:
            the distance between fingertips

        ## output
        fetchnp:
            the nodepath of this fetch hand

        author: weiwei
        date: 20160627
        '''
        self.fetchnp = NodePath("fetchhnd")
        self.handnp = self.fetchnp
        self.jawwidth = jawwidth

        this_dir, this_filename = os.path.split(__file__)
        fetchbasepath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "gripper_link.obj"))
        fetchrfingerpath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "r_gripper_finger_link.obj"))
        fetchlfingerpath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "l_gripper_finger_link.obj"))
        # print fetchrfingerpath
        fetchbase = NodePath("gripper_link")
        fetchrfinger = NodePath("r_gripper_finger_link")
        fetchlfinger = NodePath("l_gripper_finger_link")

        # loader is a global variable defined by panda3d
        fetchbasel = loader.loadModel(fetchbasepath)
        fetchrfingerl = loader.loadModel(fetchrfingerpath)
        fetchlfingerl = loader.loadModel(fetchlfingerpath)
        fetchbasel.setScale(1000)
        fetchrfingerl.setScale(1000)
        fetchlfingerl.setScale(1000)

        # base
        fetchbasel.instanceTo(fetchbase)
        fetchbase.setPos(30,0,0)

        # left and right finger
        fetchlfingerl.instanceTo(fetchlfinger)
        fetchlfinger.setPos(0, 0, 116.85)
        fetchlfinger.reparentTo(fetchbase)
        fetchrfingerl.instanceTo(fetchrfinger)
        fetchrfinger.setPos(0, 0, -116.85)
        fetchrfinger.reparentTo(fetchbase)

        # rotate to x, y, z coordinates (this one rotates the base, not the self.fetchnp)
        # the default x direction is facing the ee, the default z direction is facing downward
        # execute this file to see the default pose
        fetchbase.setMat(pandageom.cvtMat4(rm.rodrigues([1,0,0], 90))*pandageom.cvtMat4(rm.rodrigues([0,0,1], 180))*fetchbase.getMat())
        fetchbase.reparentTo(self.fetchnp)
        self.setJawwidth(jawwidth)

        self.__jawwidthopen = 100
        self.__jawwidthclosed = 0

    @property
    def jawwidthopen(self):
        # read-only property
        return self.__jawwidthopen

    @property
    def jawwidthclosed(self):
        # read-only property
        return self.__jawwidthclosed

    def setJawwidth(self, jawwidth):
        '''
        set the jawwidth of fetchhnd
        the formulea is deduced on a note book

        ## input
        fetchhnd:
            nodepath of a fetch hand
        jawwidth:
            the width of the jaw

        author: weiwei
        date: 20160627
        '''
        assert(jawwidth <= 100)
        assert(jawwidth >= 0)

        self.jawwidth = jawwidth

        # right finger
        r_gripper_finger = self.fetchnp.find("**/r_gripper_finger_link")
        r_gripper_fingerPos = r_gripper_finger.getPos()
        r_gripper_finger.setPos(r_gripper_fingerPos[0], r_gripper_fingerPos[1], r_gripper_fingerPos[2] + jawwidth/2)


        # left finger
        l_gripper_finger = self.fetchnp.find("**/l_gripper_finger_link")
        l_gripper_fingerPos = l_gripper_finger.getPos()
        l_gripper_finger.setPos(l_gripper_fingerPos[0], l_gripper_fingerPos[1], l_gripper_fingerPos[2] - jawwidth/2)


    def setPos(self, npvec3):
        """
        set the pose of the hand
        changes self.fetchnp

        :param npvec3
        :return:
        """

        self.fetchnp.setPos(npvec3)

    def getPos(self):
        """
        set the pose of the hand
        changes self.fetchnp

        :param npvec3
        :return:
        """

        return self.fetchnp.getPos()

    def setMat(self, npmat4):
        """
        set the translation and rotation of a robotiq hand
        changes self.fetchnp

        :param npmat4: follows panda3d, a LMatrix4f matrix
        :return: null

        date: 20161109
        author: weiwei
        """

        self.fetchnp.setMat(npmat4)

    def getMat(self):
        """
        get the rotation matrix of the hand

        :return: npmat4: follows panda3d, a LMatrix4f matrix

        date: 20161109
        author: weiwei
        """

        return self.fetchnp.getMat()

    def reparentTo(self, nodepath):
        """
        add to scene, follows panda3d

        :param nodepath: a panda3d nodepath
        :return: null

        date: 20161109
        author: weiwei
        """
        self.fetchnp.reparentTo(nodepath)

    def removeNode(self):
        """

        :return:
        """

        self.fetchnp.removeNode()

    def lookAt(self, direct0, direct1, direct2):
        """
        set the Y axis of the hnd

        author: weiwei
        date: 20161212
        """

        self.fetchnp.lookAt(direct0, direct1, direct2)

    def plot(self, nodepath, pos=None, ydirect=None, zdirect=None, rgba=None):
        '''
        plot the hand under the given nodepath

        ## input
        nodepath:
            the parent node this hand is going to be attached to
        pos:
            the position of the hand
        ydirect:
            the y direction of the hand
        zdirect:
            the z direction of the hand
        rgba:
            the rgba color

        ## note:
            dot(ydirect, zdirect) must be 0

        date: 20160628
        author: weiwei
        '''

        if pos is None:
            pos = Vec3(0,0,0)
        if ydirect is None:
            ydirect = Vec3(0,1,0)
        if zdirect is None:
            zdirect = Vec3(0,0,1)
        if rgba is None:
            rgba = Vec4(1,1,1,0.5)

        # assert(ydirect.dot(zdirect)==0)

        placeholder = nodepath.attachNewNode("fetchholder")
        self.fetchnp.instanceTo(placeholder)
        xdirect = ydirect.cross(zdirect)
        transmat4 = Mat4()
        transmat4.setCol(0, xdirect)
        transmat4.setCol(1, ydirect)
        transmat4.setCol(2, zdirect)
        transmat4.setCol(3, pos)
        self.fetchnp.setMat(transmat4)
        placeholder.setColor(rgba)

if __name__=='__main__':

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())

        return task.cont

    base = pandactrl.World()
    fetchhnd = designpattern.singleton(Fetch_gripper)
    fetchhnd.setJawwidth(0)
    hndpos = Vec3(0,0,0)
    ydirect = Vec3(0,1,0)
    zdirect = Vec3(0,0,1)
    fetchhnd.plot(base.render, pos=hndpos, ydirect=ydirect, zdirect=zdirect)

    axis = loader.loadModel('zup-axis.egg')
    axis.reparentTo(base.render)
    axis.setPos(hndpos)
    axis.setScale(50)
    axis.lookAt(hndpos+ydirect)


    bullcldrnp = base.render.attachNewNode("bulletcollider")
    base.world = BulletWorld()

    # hand base
    g_hand_np = fetchhnd.fetchnp.find("**/gripper_link").find("**/+GeomNode")
    g_hand = g_hand_np.node().getGeom(0)
    g_hand_transform = g_hand_np.getTransform(base.render)
    g_hand_mesh = BulletTriangleMesh()
    g_hand_mesh.addGeom(g_hand)
    handbullnode = BulletRigidBodyNode('g_hand')
    handbullnode.addShape(BulletTriangleMeshShape(g_hand_mesh, dynamic=True), g_hand_transform)
    hand_collidernp=bullcldrnp.attachNewNode(handbullnode)
    base.world.attachRigidBody(handbullnode)
    hand_collidernp.setCollideMask(BitMask32.allOn())

    # left finger
    g_left_np = fetchhnd.fetchnp.find("**/l_gripper_finger_link").find("**/+GeomNode")
    g_left = g_left_np.node().getGeom(0)
    g_left_transform = g_left_np.getTransform(base.render)
    g_left_mesh = BulletTriangleMesh()
    g_left_mesh.addGeom(g_left)
    # leftbullnode = BulletRigidBodyNode('g_left')
    # leftbullnode.addShape(BulletTriangleMeshShape(g_left_mesh, dynamic=True), g_left_transform)
    # left_collidernp=bullcldrnp.attachNewNode(leftbullnode)
    # base.world.attachRigidBody(leftbullnode)
    # left_collidernp.setCollideMask(BitMask32.allOn())

    # right finger
    g_right_np = fetchhnd.fetchnp.find("**/r_gripper_finger_link").find("**/+GeomNode")
    g_right = g_right_np.node().getGeom(0)
    g_right_transform = g_right_np.getTransform(base.render)
    g_right_mesh = BulletTriangleMesh()
    g_right_mesh.addGeom(g_right)

    # rightbullnode = BulletRigidBodyNode('g_right')
    # rightbullnode.addShape(BulletTriangleMeshShape(g_right_mesh, dynamic=True), g_right_transform)
    # right_collidernp=bullcldrnp.attachNewNode(rightbullnode)
    # base.world.attachRigidBody(rightbullnode)
    # right_collidernp.setCollideMask(BitMask32.allOn())



    # ilkbullnode = BulletRigidBodyNode('gilk')
    # ilkbullnode.addShape(BulletTriangleMeshShape(g_right_mesh, dynamic=True), g_right_transform)
    # ilkbullnode.addShape(BulletTriangleMeshShape(g_left_mesh, dynamic=True), g_left_transform)
    # ilkcollidernp=bullcldrnp.attachNewNode(ilkbullnode)
    # base.world.attachRigidBody(ilkbullnode)
    # ilkcollidernp.setCollideMask(BitMask32.allOn())

    # base.taskMgr.add(updateworld, "updateworld", extraArgs=[base.world], appendTask=True)
    # result = base.world.contactTestPair(handbullnode, ilkbullnode)
    # print result
    # print result.getContacts()
    # import pandaplotutils.pandageom as pandageom
    # for contact in result.getContacts():
    #     cp = contact.getManifoldPoint()
    #     # print cp.getLocalPointA()
    #     pandageom.plotSphere(base.render, pos=cp.getLocalPointA(), radius=0.01, rgba=Vec4(1,0,0,1))

    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNode.showConstraints(True)
    # debugNode.showBoundingBoxes(False)
    # debugNode.showNormals(False)
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # # debugNP.show()

    # base.world.setDebugNode(debugNP.node())

    base.run()