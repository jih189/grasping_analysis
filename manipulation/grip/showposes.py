#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
import itertools
from utils import dbcvt as dc

from database import dbaccess as db
import matplotlib.pyplot as plt
from fetch_gripper import fetch_grippernm
from utils import collisiondetection as cd

if __name__ == '__main__':
    this_dir, this_filename = os.path.split(__file__)
    handpkg = fetch_grippernm
    gdb = db.GraspDB()

    object_name = "bottle"

    obj_id = gdb.loadIdObject(object_name)
    print "object", object_name ,"id =", obj_id

    obj_placements = gdb.loadFreeTabletopPlacement(object_name)

    for p in obj_placements:
        print(p)




