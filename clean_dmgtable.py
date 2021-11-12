#!/usr/bin/python

import MySQLdb

db = MySQLdb.connect("localhost", "cogrob", "cogrob", "freegrip")

cursor = db.cursor()

# drop table if it already exist
cursor.execute("drop table if exists dmgs")
cursor.execute("drop table if exists angleranges")
cursor.execute("drop table if exists grasps")
cursor.execute("drop table if exists dmgedges")

# create tables
sql = """create table dmgs (
                    iddmg int primary key,
                    idobject int,
                    placementpose char(250),
                    planevector char(140)
                )"""

cursor.execute(sql)

sql = """create table angleranges (
                    idanglerange int primary key,
                    iddmg int,
                    grasporder char(250)
                )"""

cursor.execute(sql)

sql = """create table grasps (
                    idgrasp int primary key,
                    idanglerange int,
                    pose char(250),
                    jawwidth char(20)
                )"""

cursor.execute(sql)

sql = """create table dmgedges (
                    iddmgedge int primary key auto_increment,
                    iddmg int,
                    idanglerange1 int,
                    idanglerange2 int,
                    connectid1 int,
                    connectid2 int
                )"""

cursor.execute(sql)

db.close()

