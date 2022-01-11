#!/usr/bin/python3

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import actionlib
import control_msgs.msg
import rospy
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as rot

from ur5_inverse import ur5Inverse
from ur5_direct import ur5Direct

#Contains current joint angles -> updated every time
#['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
current_pos = [0, 1.5, 1.0, 0.0, 0.0, 0]
# Contains gripper aperture -> 0.0 to 0.8
current_aperture = [0]

def rotm2eul(R):
    #assert(isRotationMatrix(R))

    sy = np.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    
    return np.array([z, y, x])

def eul2rotm(theta):
    Rx = np.array([[1,   0,  0],
                  [0,   math.cos(theta[0]), -math.sin(theta[0])],
                  [0,   math.sin(theta[0]),   math.cos(theta[0])]])

    Ry = np.array([[math.cos(theta[1]),   0,  math.sin(theta[1])],
                  [0,   1, 0],
                  [-math.sin(theta[1]),   0,   math.cos(theta[1])]])

    Rz = np.array([[math.cos(theta[2]),   -math.sin(theta[2]),  0],
                  [math.sin(theta[2]),   math.cos(theta[2]), 0],
                  [0,   0,   1]])

    R = np.dot(Rz, np.dot(Ry, Rx))
    return R

def gripper_client(value):
    threshold = 0.001
    # Create an action client
    client = actionlib.SimpleActionClient(
        '/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()
    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()

    goal.command.position = value   # From 0.0 to 0.8
    goal.command.max_effort = -1.0  # Do not limit the effort

    client.send_goal(goal)
    client.wait_for_result()

    #while not (current_aperture[0]>value-threshold and current_aperture[0]<value+threshold):
        #print("")
    
        

def moveTo(xef, phief):
    pub = rospy.Publisher('/trajectory_controller/command', JointTrajectory, queue_size=10)
    
    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']

    rate = rospy.Rate(10)

    #Set current joint angles
    TH0 = current_pos
    xe0, Re = ur5Direct(TH0)
    phie0 = np.transpose(rotm2eul(Re))

    #
    if xef[1] > 0.2:
        xef[1] -= 0.01
    else:
        xef[1] -= 0.025

    if xef[1] > 0.1:
        xef[0] +=0.02
    elif xef[1] > -0.1:
        xef[0] +=0.01

    xef = np.transpose(xef)
    phief = np.transpose(phief)

    #Functions that can be used to do point to point
    xe = lambda t: np.dot(t,xef) + np.dot((1-t),xe0)
    phie = lambda t: np.dot(t,phief) + np.dot((1-t),phie0)

    x = xe(1)
    phi = phie(1)
    phi = np.transpose(phi)
    Th = ur5Inverse(x, rot.from_euler('ZYX', [phi[0], phi[1], phi[2]]).as_dcm()) # A seconda da che versioni di scipy: as_dcm() o as_matrix()
    while not rospy.is_shutdown():

        threshold = 0.005
        if (current_pos[0]>Th[6][0]-threshold and current_pos[0]<Th[6][0]+threshold) and (current_pos[1]>Th[6][1]-threshold and current_pos[1]<Th[6][1]+threshold) and (current_pos[2]>Th[6][2]-threshold and current_pos[2] < Th[6][2]+threshold) and (current_pos[3]>Th[6][3]-threshold and current_pos[3] < Th[6][3]+threshold) and (current_pos[4]>Th[6][4]-threshold and current_pos[4] < Th[6][4]+threshold) and (current_pos[5]>Th[6][5]-threshold and current_pos[5] < Th[6][5]+threshold):
            break

        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()

        #pts.positions = [0, -1.5, 1.0, 0, 0, 0]
        pts.positions = [Th[7][0], Th[6][1], Th[6][2], Th[6][3], Th[6][4], Th[6][5]]
        #print(pts.positions)
        pts.time_from_start = rospy.Duration(0.4)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)

def jointState(msg):
    # ------
    # JointState structure
    # name: -elbow_joint -robotiq_85_left_knuckle_joint -shoulder_lift_joint -shoulder_pan_joint -wrist_1_joint -wrist_2_joint -wrist_3_joint
    # ------
    current_pos[0] = msg.position[3]
    current_pos[1] = msg.position[2]
    current_pos[2] = msg.position[0]
    current_pos[3] = msg.position[4]
    current_pos[4] = msg.position[5]
    current_pos[5] = msg.position[6]
    current_aperture[0] = msg.position[1]

def main():
    rospy.init_node('send_joints')
    sub = rospy.Subscriber('/joint_states', JointState, jointState)
    
    x=0.375599987615098
    y= -0.13955389288297085

    
    #Final position of end effector
    xef = np.array([x, y, 0.35])
    #Final orientation of end effector
    phief = np.array([0.0, np.pi, 0]) #first value from 0.0 to 3.14

    gripper_client(0.0) 

    #Calculate joint angle matrix
    Th = moveTo(xef, phief) 

    xef = np.array([x, y, 0.22])
    Th = moveTo(xef, phief) 

    gripper_client(0.51)
    time.sleep(6)

    xef = np.array([0.3, -0.7, 0.3])
    Th = moveTo(xef, phief) 

    gripper_client(0.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
