#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random

pos = Pose(Point(random.uniform(0.3, 0.8), random.uniform(-0.45, 0.45), 0.775), Quaternion(0,0,random.uniform(-2.0, 2.0), 0.1))
print(pos)
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client(model_name='test'+str(random.random()), 
    model_xml=open('src/ur5/ur5_gazebo/models/lego_1/model.sdf', 'r').read(),
    robot_namespace='/foo',
    initial_pose=pos,
    reference_frame='world')
