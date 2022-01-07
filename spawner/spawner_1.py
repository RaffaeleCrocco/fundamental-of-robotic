#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random

blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']

pos = Pose(Point(random.uniform(0.3, 0.76), random.uniform(-0.42, 0.42), 0.775), Quaternion(0,0,random.uniform(-2.0, 2.0), 0.0))
print(pos)
brick=blocks[random.randint(0,10)]
print(brick)
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name='test'+str(random.random()), 
    model_xml=open('src/ur5/ur5_gazebo/models/'+brick+'/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')
