#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
positions = []

for i in range(11):
    #Generate random position
    pos = Pose(Point(random.uniform(0.4, 0.85), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), 0.0))
    
    #Get a random lego block from all legos
    brick=blocks[i]
    print(pos)
    print(brick)
    #Call rospy spawn function to spawn objects in gazebo
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name='b-'+str(brick), 
        model_xml=open('src/ur5/ur5_gazebo/models/'+brick+'/model.sdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=pos,
        reference_frame='world')
