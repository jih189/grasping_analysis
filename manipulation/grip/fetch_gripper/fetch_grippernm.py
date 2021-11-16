import math
import os

import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletDebugNode
# from panda3d.bullet import BulletRigidBodyNode
# from panda3d.bullet import BulletTriangleMesh
# from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *
from utils import collisiondetection as cd

# from utils import designpattern

class Fetch_gripperNM():
    '''
    use utils.designpattern.singleton() to get a single instance of this class
    '''

    def __init__(self, jawwidth=0, hndcolor=None):
        '''
        load the fetch gripper model, set jawwidth and return a nodepath

        NOTE: the setColor function is only useful when the models dont have any materials

        ## input
        pandabase:
            the showbase() object
        jawwidth:
            the distance between fingertips

        ## output
        handnp:
            the nodepath of this fetch hand

        author: weiwei
        date: 20160627
        '''
        self.handnp = NodePath("fetchhnd")

        self.jawwidth = jawwidth

        this_dir, _ = os.path.split(__file__)
        fetchpalmpath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "gripper_link.egg"))
        fetchrfingerpath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "r_gripper_finger_link.egg"))
        fetchlfingerpath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "l_gripper_finger_link.egg"))

        fetchpalm = NodePath("gripper_link")
        fetchrfinger = NodePath("r_gripper_finger_link")
        fetchlfinger = NodePath("l_gripper_finger_link")
        fetchrfingertip = NodePath("r_gripper_fingertip_link")
        fetchlfingertip = NodePath("l_gripper_fingertip_link")

        # loader is a global variable defined by panda3d
        fetchrfingerl = loader.loadModel(fetchrfingerpath)
        fetchlfingerl = loader.loadModel(fetchlfingerpath)
        fetchpalml = loader.loadModel(fetchpalmpath)

        # set the palm node path        
        fetchpalml.instanceTo(fetchpalm)
        fetchpalm.setPos(0,0,0)

        # set the left finger node path
        fetchlfingerl.instanceTo(fetchlfinger)
        fetchlfinger.setPos(0, -116.85, 0)
        fetchlfingertip.setPos(20, -50, 0)   
        fetchlfinger.reparentTo(fetchpalm)
        fetchlfingertip.reparentTo(fetchpalm)

        # set the right finger node path
        fetchrfingerl.instanceTo(fetchrfinger)
        fetchrfinger.setPos(0, 116.85, 0)
        fetchrfingertip.setPos(20, 50, 0)
        fetchrfinger.reparentTo(fetchpalm)
        fetchrfingertip.reparentTo(fetchpalm)

        # draw the fingertips in frame
        pandageom.plotSphere(fetchrfingertip, pos=Point3(0, 0, 0), radius=5, rgba=Vec4(1,0,0,1))
        pandageom.plotSphere(fetchlfingertip, pos=Point3(0, 0, 0), radius=5, rgba=Vec4(1,0,0,1))

        # set color if need
        if hndcolor is None:
            pass
        else:
            fetchpalm.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
            fetchlfinger.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])
            fetchrfinger.setColor(hndcolor[0],hndcolor[1],hndcolor[2],hndcolor[3])

        fetchpalm.setTransparency(TransparencyAttrib.MAlpha)
        fetchlfinger.setTransparency(TransparencyAttrib.MAlpha)
        fetchrfinger.setTransparency(TransparencyAttrib.MAlpha)

        # fetchpalm.setMat(pandageom.cvtMat4(rm.rodrigues([0,0,1], 180))*fetchpalm.getMat())
        fetchpalm.reparentTo(self.handnp)

        # right finger
        self.r_gripper_fingerPos_init = self.handnp.find("**/r_gripper_finger_link").getPos()
        self.l_gripper_fingerPos_init = self.handnp.find("**/l_gripper_finger_link").getPos()

        self.r_gripper_fingertipPos_init = self.handnp.find("**/r_gripper_fingertip_link").getPos()
        self.l_gripper_fingertipPos_init = self.handnp.find("**/l_gripper_fingertip_link").getPos()
        
        self.setJawwidth(jawwidth)

        self.__jawwidthopen = 100.0
        self.__jawwidthclosed = 0.0

    @property
    def jawwidthopen(self):
        # read-only property
        return self.__jawwidthopen

    @property
    def jawwidthclosed(self):
        # read-only property
        return self.__jawwidthclosed

    # get the two fingertips in the hand link, you can use this function to get the fingertips direction
    def getFingerTips(self):
        right_contact_point = self.handnp.find("**/r_gripper_fingertip_link").getPos()
        left_contact_point = self.handnp.find("**/l_gripper_fingertip_link").getPos()
        return right_contact_point, left_contact_point

    def setJawwidth(self, jawwidth):
        '''
        set the jawwidth of fetch gripper
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
        r_gripper_finger = self.handnp.find("**/r_gripper_finger_link")
        r_gripper_finger.setPos(self.r_gripper_fingerPos_init[0], self.r_gripper_fingerPos_init[1] + jawwidth / 2, self.r_gripper_fingerPos_init[2])

        # left finger
        l_gripper_finger = self.handnp.find("**/l_gripper_finger_link")
        l_gripper_finger.setPos(self.l_gripper_fingerPos_init[0], self.l_gripper_fingerPos_init[1] - jawwidth / 2, self.l_gripper_fingerPos_init[2])

    def setColor(self, rgbacolor=[1,0,0,.1]):
        """
        set the color of the hand

        :param rgbacolor:
        :return:

        author: weiwei
        date: 20161212
        """

        # base
        s_fetchbase = self.handnp.find("**/gripper_link")
        s_fetchbase.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])

        # right finger
        s_right_finger = self.handnp.find("**/r_gripper_finger_link")
        s_right_finger.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])


        # left finger
        s_left_finger = self.handnp.find("**/l_gripper_finger_link")
        s_left_finger.setColor(rgbacolor[0], rgbacolor[1], rgbacolor[2], rgbacolor[3])

    def setPos(self, npvec3):
        """
        set the pose of the hand
        changes self.handnp

        :param npvec3
        :return:
        """

        self.handnp.setPos(npvec3)

    def getPos(self):
        """
        set the pose of the hand
        changes self.handnp

        :return:npvec3
        """

        return self.handnp.getPos()

    def setMat(self, nodepath = None, pandanpmat4 = Mat4.identMat()):
        """
        set the translation and rotation of a robotiq hand
        changes self.handnp

        :param npmat4: follows panda3d, a LMatrix4f matrix
        :return: null

        date: 20161109
        author: weiwei
        """

        self.handnp.setMat(pandanpmat4)

    def getMat(self):
        """
        get the rotation matrix of the hand

        :return: npmat4: follows panda3d, a LMatrix4f matrix

        date: 20161109
        author: weiwei
        """

        return self.handnp.getMat()

    def reparentTo(self, nodepath):
        """
        add to scene, follows panda3d

        :param nodepath: a panda3d nodepath
        :return: null

        date: 20161109
        author: weiwei
        """
        self.handnp.reparentTo(nodepath)

    def removeNode(self):
        """

        :return:
        """

        self.handnp.removeNode()

    def detachNode(self):
        """

        :return:
        """

        self.handnp.detachNode()

    def lookAt(self, direct0, direct1, direct2):
        """
        set the Y axis of the hnd

        author: weiwei
        date: 20161212
        """

        self.handnp.lookAt(direct0, direct1, direct2)

    def plot(self, pandabase, nodepath=None, pos=None, ydirect=None, zdirect=None, rgba=None):
        '''
        plot the hand under the given nodepath

        ## input
        pandabase:
            a showbase instance
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

        if nodepath is None:
            nodepath = pandabase.render
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
        self.handnp.instanceTo(placeholder)
        xdirect = ydirect.cross(zdirect)
        transmat4 = Mat4()
        transmat4.setCol(0, xdirect)
        transmat4.setCol(1, ydirect)
        transmat4.setCol(2, zdirect)
        transmat4.setCol(3, pos)
        self.handnp.setMat(transmat4)
        placeholder.setColor(rgba)


def newHandNM(jawwidth = 100, hndcolor = None):
    return Fetch_gripperNM(jawwidth, hndcolor)

def newHandFgrpcc():
    this_dir, _ = os.path.split(__file__)
    handfgrpccpath = Filename.fromOsSpecific(os.path.join(this_dir, "fetchegg", "fetch_tip_precc.egg"))
    handfgrpcc = loader.loadModel(handfgrpccpath)
    return handfgrpcc

def getHandName():
    return "fetch"

if __name__=='__main__':

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont

    base = pandactrl.World(camp=[0,0,600], lookatp=[0,0,0], focusLength=600)
    fetchhnd = Fetch_gripperNM(hndcolor=[.5,.5,0.5,.7])
    # test
    fetchhnd.reparentTo(base.render)
    fetchhnd.setJawwidth(0)

    axis = loader.loadModel('zup-axis.egg')
    axis.setScale(10)
    axis.reparentTo(base.render)

    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # base.world = BulletWorld()

    # # hand base
    # # fetchhnd.handnp.find("**/gripper_link").showTightBounds()
    # g_hand_np = fetchhnd.handnp.find("**/gripper_link").find("**/+GeomNode")
    # g_hand = g_hand_np.node().getGeom(0)
    # g_hand_transform = g_hand_np.getTransform(base.render)
    # g_hand_mesh = BulletTriangleMesh()
    # g_hand_mesh.addGeom(g_hand)
    # handbullnode = BulletRigidBodyNode('g_hand')
    # handbullnode.addShape(BulletTriangleMeshShape(g_hand_mesh, dynamic=True), g_hand_transform)
    # hand_collidernp=bullcldrnp.attachNewNode(handbullnode)
    # base.world.attachRigidBody(handbullnode)
    # hand_collidernp.setCollideMask(BitMask32.allOn())
    # fetchhnd.handnp.find("**/gripper_link").showTightBounds()

    # # left finger
    # g_left_np = fetchhnd.handnp.find("**/l_gripper_finger_link").find("**/+GeomNode")
    # g_left = g_left_np.node().getGeom(0)
    # g_left_transform = g_left_np.getTransform(base.render)
    # g_left_mesh = BulletTriangleMesh()
    # g_left_mesh.addGeom(g_left)
    # leftbullnode = BulletRigidBodyNode('g_left')
    # leftbullnode.addShape(BulletTriangleMeshShape(g_left_mesh, dynamic=True), g_left_transform)
    # left_collidernp=bullcldrnp.attachNewNode(leftbullnode)
    # base.world.attachRigidBody(leftbullnode)
    # left_collidernp.setCollideMask(BitMask32.allOn())
    # fetchhnd.handnp.find("**/l_gripper_finger_link").showTightBounds()

    # # right finger
    # g_right_np = fetchhnd.handnp.find("**/r_gripper_finger_link").find("**/+GeomNode")
    # g_right = g_right_np.node().getGeom(0)
    # g_right_transform = g_right_np.getTransform(base.render)
    # g_right_mesh = BulletTriangleMesh()
    # g_right_mesh.addGeom(g_right)
    # rightbullnode = BulletRigidBodyNode('g_right')
    # rightbullnode.addShape(BulletTriangleMeshShape(g_right_mesh, dynamic=True), g_right_transform)
    # right_collidernp=bullcldrnp.attachNewNode(rightbullnode)
    # base.world.attachRigidBody(rightbullnode)
    # right_collidernp.setCollideMask(BitMask32.allOn())
    # fetchhnd.handnp.find("**/r_gripper_finger_link").showTightBounds()


    # ilkbullnode = BulletRigidBodyNode('gilk')
    # ilkbullnode.addShape(BulletTriangleMeshShape(g_right_mesh, dynamic=True), g_right_transform)
    # ilkbullnode.addShape(BulletTriangleMeshShape(g_left_mesh, dynamic=True), g_left_transform)
    # ilkcollidernp=bullcldrnp.attachNewNode(ilkbullnode)
    # base.world.attachRigidBody(ilkbullnode)
    # ilkcollidernp.setCollideMask(BitMask32.allOn())


    # base.taskMgr.add(updateworld, "updateworld", extraArgs=[base.world], appendTask=True)
    # result = base.world.contactTestPair(handbullnode, ilkbullnode)

    # import pandaplotutils.pandageom as pandageom
    # for contact in result.getContacts():
    #     cp = contact.getManifoldPoint()
    #     pandageom.plotSphere(base.render, pos=cp.getLocalPointA(), radius=0.01, rgba=Vec4(1,0,0,1))

    # pandageom.plotAxisSelf(base.render, spos = Vec3(0,0,0), length=0.3, thickness=0.01)

    

    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNode.showConstraints(True)
    # debugNode.showBoundingBoxes(False)
    # debugNode.showNormals(False)
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()

    # base.world.setDebugNode(debugNP.node())

    # bulletObj = BulletWorld()
    # bulletObj.attachRigidBody(cd.genCollisionMeshMultiNp(fetchhnd.palmnp))

    # show the collision net

    # bulletworldhp = BulletWorld()

    # # plane to remove hand
    # planebullnode = cd.genCollisionPlane(offset=0)
    # bulletworldhp.attachRigidBody(planebullnode)

    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNode.showConstraints(True)
    # debugNode.showBoundingBoxes(False)
    # debugNode.showNormals(False)
    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    # bulletworldhp.setDebugNode(debugNP.node())
    # base.taskMgr.add(updateworld, "updateworld", extraArgs=[bulletworldhp], appendTask=True)

    base.run()