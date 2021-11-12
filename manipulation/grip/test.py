#!/usr/bin/python
import sys
from database import dbaccess as db
from ffregrasp import PandaPosMax_t_PosMat, PosMat_t_PandaPosMax
from utils import dbcvt as dc

gdb = db.GraspDB()   #SQL grasping database interface

sql = "SELECT * FROM object WHERE object.name LIKE '%s'" % "cup"
result = gdb.execute(sql)
if not result:
    print "please add the object name to the table first!!"
    raise Exception("the table does not contain the object!")
else:
    objectId = int(result[0][0])

sql = "SELECT grasppose, jawwidth FROM targetgrasps WHERE idobject = '%d'" % objectId
targetgrasp_result = gdb.execute(sql)

target_grasps = []
for grasppose, jawwidth in targetgrasp_result:
    target_grasps.append((PandaPosMax_t_PosMat(dc.strToMat4(grasppose)), float(jawwidth) / 1000))

