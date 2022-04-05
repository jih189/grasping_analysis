#!/usr/bin/python

import MySQLdb

db = MySQLdb.connect("localhost", "cogrob", "cogrob", "freegrip")

cursor = db.cursor()

# drop table if it already exist
cursor.execute("drop table if exists object")
cursor.execute("drop table if exists freeairgrip")
cursor.execute("drop table if exists hand")
cursor.execute("drop table if exists freetabletopplacement")
cursor.execute("drop table if exists freetabletopgrip")
cursor.execute("drop table if exists dmgs")
cursor.execute("drop table if exists angleranges")
cursor.execute("drop table if exists grasps")
cursor.execute("drop table if exists dmgedges")
cursor.execute("drop table if exists targetgrasps")
cursor.execute("drop table if exists initgrasps")

# # create tables
sql = """create table initgrasps (
                    idinitgrasp int primary key auto_increment,
                    idobject int,
                    grasppose char(250),
                    jawwidth char(20)
                )"""

cursor.execute(sql)

sql = """create table targetgrasps (
                    idtargetgrasp int primary key auto_increment,
                    idobject int,
                    grasppose char(250),
                    jawwidth char(20)
                )"""

cursor.execute(sql)

sql = """create table dmgs (
                    iddmg int primary key,
                    idobject int,
                    placementpose char(250),
                    placementid int,
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

sql = """create table freetabletopgrip (
                    idfreetabletopgrip int primary key auto_increment,
                    idfreetabletopplacement int,
                    idfreeairgrip int,
                    contactpoint0 char(70),
                    contactpoint1 char(70),
                    contactnormal0 char(70),
                    contactnormal1 char(70),
                    rotmat char(250),
                    jawwidth char(20)
                )"""

cursor.execute(sql)

sql = """create table freetabletopplacement (
                    idfreetabletopplacement int primary key auto_increment,
                    rotmat char(250),
                    placement int,
                    idobject int
                )"""
# placement means what kind of placement it is,
# 0: stable placement
# 1: unstable placement (ff)

cursor.execute(sql)



sql = """create table object (
                    idobject int primary key,
                    name char(20)
                )"""

cursor.execute(sql)

sql = """create table freeairgrip(
                    idfreeairgrip int primary key auto_increment,
                    idobject int,
                    contactpnt0 char(70),
                    contactpnt1 char(70),
                    contactnormal0 char(70),
                    contactnormal1 char(70),
                    rotmat char(250),
                    jawwidth char(20),
                    idhand int
            )"""
cursor.execute(sql)

sql = """create table hand(
                    idhand int primary key,
                    name char(20)
            )"""
cursor.execute(sql)

# add the object's name and id to the table

sql = """insert into object(idobject, name) values (0,'ttube')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (1,'cuboid')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (2,'book')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (3,'good_book')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (4,'cylinder')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (5,'cup')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (6,'box')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (7,'almonds_can')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (8,'Lshape')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (9,'bottle')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (10,'can')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (11,'Ushape')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into object(idobject, name) values (12,'Hshape')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()

sql = """insert into hand(idhand, name) values (0,'fetch')"""
try:
    cursor.execute(sql)
    db.commit()
except:
    db.rollback()


db.close()

