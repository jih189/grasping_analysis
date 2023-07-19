This tool is used to generate a set of object's stable placement with its feasible grasp poses. Those result will be stored into a sql database.

----------------------------------------------------------------------------------------------
*** install mysql and create both database and user ***
Install mysql, please follow the tutorial in https://ubuntu.com/server/docs/databases-mysql

after that, type "sudo mysql" to enter the sql, and create a database called "freegrip"
	create database freegrip;
Then, you can check the database by
	show databases;

create a user in the database by
	create user 'cogrob'@'localhost' identified by 'cogrob';
where the user name is 'cogrob' and the password is also 'cogrob'.

then grant all privileges of freegrip to user 'cogrob' by following command
	grant all on freegrip.* to 'cogrob'@'localhost';

After that, you can exit mysql, and enter the database as user by
	mysql -u cogrob -p
where the password is 'cogrob'

-----------------------------------------------------------------------------------------------
*** install related tools ***
Install related tools by following commands(you may need to use python -m pip install):
```
	pip install Panda3D
	pip install shapely
	pip install Rtree
	pip install networkx
	pip install scikit-learn
	apt install python-tk
	apt install python-rtree
```
------------------------------------------------------------------------------------------------
*** Only want the grasp poses over the object ***
In this case, you do not need to install database, but you still need to install all related tools above.

Assume you are in the grasping_analysis directory.

1. To install this package, type 
```
python setup.py install --user
```

2. Save the mesh file as .stl format into manipulation/grip/objects directory.

3. Enter manipulation/grip/generate_grasp_poses.py. Modify following code for your object
```
objpath = os.path.join(this_dir, "objects", "cup.stl")
```
by changing 'cup' to the name you want.

4. run
```
python generate_grasp_poses.py
```
This script will generate a file [object].npz. In this file, it is a list of grasp poses in the object frame for fetch gripper.

------------------------------------------------------------------------------------------------
*** generate placement and grasp poses ***
Assume you are in the grasping_analysis directory.

1. To install this package, type
```
python setup.py install --user
```
1. add the object name and id into the database by adding the following code before "db.close()"

	sql = """insert into object(idobject, name) values ([#id],[object name])"""
	try:
    		cursor.execute(sql)
    		db.commit()
	except:
    		db.rollback()

2. run the following command to generate the database.
	python create_tables.py

3. add the object mesh(.stl) file into to manipulation/grip/objects. (the unit in those file should be mm)

4. modify the object name in manipulation/grip/freegrip_fetch.py
	"obj_path = os.path.join(this_dir, "objects", "bottle.stl")" -> "obj_path = os.path.join(this_dir, "objects", "[object name].stl")" 

5. generate possible grasp poses in the object coordinate by following command
	python freegrip_fetch.py

6. then it should show a window to demonstrate all the posssible grasp poses in the object coordinate

7. modify the object name in manipulation/grip/freetabletopplacement.py
	"obj_path = os.path.join(this_dir, "objects", "bottle.stl")" -> "obj_path = os.path.join(this_dir, "objects", "[object name].stl")" 

8. generate possible stable placement with its feasible grasp pose
	python freetabletopplacement.py

-------------------------------------------------------------------------------------------
*** show poses ***
Modify the object name in showposes.py first, then
by running "python showposes.py", it shows all possible placement poses in the table coordiante with a set of grasp poses feasible in that placement. By the way,
those grasp poses are in the table coordiante as well.



