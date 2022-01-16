#!/usr/bin/python3

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import actionlib
import control_msgs.msg
from std_msgs.msg import String
import rospy
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as rot

from ur5_inverse import ur5Inverse
from ur5_direct import ur5Direct

#Contains current joint angles -> updated every time
#['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
current_pos = [-0.9, -2, -1.3, 0.0, 0.0, 0]

objectives = []          # List containing all the objects detected with respective positions and orientations

blocks = np.array([['X1-Y2-Z1', 'X2-Y2-Z2', 'X1-Y3-Z2', 'X1-Y2-Z2', 'X1-Y2-Z2-CHAMFER', 'X1-Y4-Z2', 'X1-Y1-Z2', 'X1-Y2-Z2-TWINFILLET', 'X1-Y3-Z2-FILLET', 'X1-Y4-Z1', 'X2-Y2-Z2-FILLET'],
                   [0.515,      0.25,       0.515,      0.515,      0.515,              0.515,      0.515,      0.515,                  0.515,              0.515,      0.25]])

# Rotation matrix to euler angles function
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

# Euler function to rotation matrix function
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

# Controls grippers aperture
def gripper_client(value):
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

# Function used to move ur5 arm
def moveTo(xef, phief, pub, threshold):
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

    # Small position fixes for arm
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

    x = xe(1) #Using only one point -> final position
    phi = phie(1)
    phi = np.transpose(phi)
    Th = ur5Inverse(x, rot.from_euler('ZYX', [phi[0], phi[1], phi[2]]).as_matrix())
    while not rospy.is_shutdown():

        # Check if the arm is in close range, threshold of wanted position
        #threshold = 0.008
        if (current_pos[0]>Th[6][0]-threshold and current_pos[0]<Th[6][0]+threshold) and (current_pos[1]>Th[6][1]-threshold and current_pos[1]<Th[6][1]+threshold) and (current_pos[2]>Th[6][2]-threshold and current_pos[2] < Th[6][2]+threshold) and (current_pos[3]>Th[6][3]-threshold and current_pos[3] < Th[6][3]+threshold) and (current_pos[4]>Th[6][4]-threshold and current_pos[4] < Th[6][4]+threshold) and (current_pos[5]>Th[6][5]-threshold and current_pos[5] < Th[6][5]+threshold):
            break

        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()

        pts.positions = [Th[6][0], Th[6][1], Th[6][2], Th[6][3], Th[6][4], Th[6][5]]
        pts.time_from_start = rospy.Duration(0.2)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message to the topic
        pub.publish(traj)

# Function used for reading msg arriving from joint_states topic
def jointState(msg):
    # JointState structure:
    # name: -elbow_joint -robotiq_85_left_knuckle_joint -shoulder_lift_joint -shoulder_pan_joint -wrist_1_joint -wrist_2_joint -wrist_3_joint
    current_pos[0] = msg.position[3]
    current_pos[1] = msg.position[2]
    current_pos[2] = msg.position[0]
    current_pos[3] = msg.position[4]
    current_pos[4] = msg.position[5]
    current_pos[5] = msg.position[6]

# Function used for reading msgs arriving from yolo topic, giving position and orientation of all blocks detected
def yolovision(msg):
    x = str(msg)
    if (x[1] != "''") and (not objectives):                 # Checking if there is at least one lego block
        x = x.replace("\\", "")                             # Remove \ character
        x = x.replace('data: "', "")                        # Remove first cell
        x = x.replace('"', "")                              # Remove " character
        x = x.split('n')                                    # Split string where letter n is found
        x.pop()                                             # Remove last cell

        j=0
        for i in x:
            x[j] = x[j].replace('\n', "")
            objectives.append(x[j])
            j += 1


def main():
    rospy.init_node('send_joints')
    pub = rospy.Publisher('/trajectory_controller/command', JointTrajectory, queue_size=10)
    sub = rospy.Subscriber('/joint_states', JointState, jointState)
    yolo = rospy.Subscriber('/yolo', String, yolovision)
    
    # wait that main node received position from vision script
    time.sleep(1.5)
    # Start timer to see how much time it takes to pick up blocks   
    start_time = time.time()

    precise_threshold = 0.008
    generic_threshold = 0.1

    # Start picking up the blocks detected and put in objective list
    for i in range (0, len(objectives)):
        obj = objectives[i].split()
        x=float(obj[1])
        y=float(obj[2])

        gripper_client(0.0) 

        #Final position of end effector
        xef = np.array([x, y, 0.3])
        #Final orientation of end effector
        phief = np.array([float(obj[4]), np.pi, 0]) #first value from 0.0 to 3.14
        
        #Calculate joint angle matrix
        Th = moveTo(xef, phief, pub, generic_threshold)

        xef = np.array([x, y, 0.225])
        Th = moveTo(xef, phief, pub, precise_threshold) 

        gripper_client(float(blocks[1][int(obj[0])]))
        time.sleep(10)

        xef = np.array([x, y, 0.38])
        Th = moveTo(xef, phief, pub, generic_threshold) 

        xef = np.array([0.3, -0.6, 0.25])
        Th = moveTo(xef, phief, pub, precise_threshold) 

        gripper_client(0.0)

        time.sleep(1.9)

        xef = np.array([0.3, -0.6, 0.3])
        Th = moveTo(xef, phief, pub, generic_threshold)

    end_time = time.time()
    delta_time = end_time-start_time
    print("KPI 1-2/2-1:")
    print(delta_time)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
