#!/usr/bin/python3
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
import rospy
import random
import numpy as np

#Array containing all lego blocks names
blocks = ['X1-Y1-Z2', 'X1-Y2-Z1', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X1-Y4-Z2', 'X2-Y2-Z2', 'X2-Y2-Z2-FILLET']
positions = []

for i in range(11):
    f=True
    #Generate random position
    if i==0:
        pos = Pose(Point(random.uniform(0.4, 0.85), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
        positions.append(pos)
    else:
        while f==True:
            pos = Pose(Point(random.uniform(0.4, 0.85), random.uniform(-0.3, 0.3), 0.775), Quaternion(0,0,random.uniform(-3.14, 3.14), random.uniform(-1.57, 1.57)))
            for k in range(i):
                threshold = 0.12
                if np.sqrt((pos.position.x-positions[k].position.x)**2+(pos.position.y-positions[k].position.y)**2) < threshold:
                    break
                if k == i-1:
                    positions.append(pos)
                    f = False

    
    #Get a random lego block from all legos
    brick=blocks[i]
    print(pos)
    print(brick)
    #Call rospy spawn function to spawn objects in gazebo
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=''+str(brick), 
        model_xml=open('src/ur5/ur5_gazebo/models/'+brick+'/model.sdf', 'r').read(),
        robot_namespace='/foo',
        initial_pose=pos,
        reference_frame='world')
