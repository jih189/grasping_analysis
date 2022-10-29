#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
import itertools
from utils import dbcvt as dc

from pandaplotutils import pandageom as pg
from database import dbaccess as db
import matplotlib.pyplot as plt
from fetch_gripper import fetch_grippernm
from utils import dbcvt as dc 

if __name__ == '__main__':
    this_dir, this_filename = os.path.split(__file__)
    handpkg = fetch_grippernm
    gdb = db.GraspDB()

    object_name = "bottle"

    obj_id = gdb.loadIdObject(object_name)
    print "object", object_name ,"id =", obj_id

    obj_placements = gdb.loadFreeTabletopPlacement(object_name)
    sql = "SELECT idfreetabletopplacement, rotmat FROM freetabletopplacement WHERE idobject=%d" % obj_id
    result = gdb.execute(sql)
    for p_id, pose in result:
        print "------------------------------------------"
        print "placement pose"
        print p_id
        print "with pose matrix"
        print pg.mat4ToNp(dc.strToMat4(pose))

        print "list of grasp poses"

        # find all feasible grasp poses in this placement
        sql = "SELECT rotmat FROM freetabletopgrip WHERE idfreetabletopplacement=%d" % p_id
        result = gdb.execute(sql)
        for pose in result:
            print pg.mat4ToNp(dc.strToMat4(pose[0]))

