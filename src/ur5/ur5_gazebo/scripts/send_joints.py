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
current_pos = [0, -1.5, 1.0, 0.0, 0.0, 0]
# Contains gripper aperture -> 0.0 to 0.8
current_aperture = [0]

def ur5Jac(Th):
    A = [0, -0.425, -0.3922, 0, 0, 0]
    D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]

    A1=A[0]
    A2=A[1]
    A3=A[2]
    A4=A[3]
    A5=A[4]
    A6=A[5]

    D1=D[0]
    D2=D[1]
    D3=D[2]
    D4=D[3]
    D5=D[4]
    D6=D[5]

    th1 = Th[0]
    th2 = Th[1]
    th3 = Th[2]
    th4 = Th[3]
    th5 = Th[4]
    th6 = Th[5]

    J1 = np.array([[D5*(np.cos(th1)*np.cos(th5) + np.cos(th2+th3+th4)*np.sin(th1)*np.sin(th5)) + D3*np.cos(th1) + D4*np.cos(th1) - A3*np.cos(th2+th3)*np.sin(th1) - A2*np.cos(th2)*np.sin(th1) - D5*np.sin(th2+th3+th4)*np.sin(th1)],
                    [D5*(np.cos(th5)*np.sin(th1) - np.cos(th2+th3+th4)*np.cos(th1)*np.sin(th5)) + D3*np.sin(th1) + D4*np.sin(th1) + A3*np.cos(th2+th3)*np.cos(th1) + A2*np.cos(th1)*np.cos(th2) - D5*np.sin(th2+th3+th4)*np.cos(th1)],
                    [0],
                    [0],
                    [0],
                    [1]])
    
    J2 = np.array([[-np.cos(th1)*(A3*np.sin(th2 + th3) + A2*np.sin(th2) + D5*(np.sin(th2 + th3)*np.sin(th4) - np.cos(th2 + th3)*np.cos(th4)) - D5*np.sin(th5)*(np.cos(th2 + th3)*np.sin(th4) + np.sin(th2 + th3)*np.cos(th4)))],
                    [ -np.sin(th1)*(A3*np.sin(th2 + th3) + A2*np.sin(th2) + D5*(np.sin(th2 + th3)*np.sin(th4) - np.cos(th2 + th3)*np.cos(th4)) - D5*np.sin(th5)*(np.cos(th2 + th3)*np.sin(th4) + np.sin(th2 + th3)*np.cos(th4)))],
                    [ A3*np.cos(th2 + th3) - (D5*np.sin(th2 + th3 + th4 + th5))/2 + A2*np.cos(th2) + (D5*np.sin(th2 + th3 + th4 - th5))/2 + D5*np.sin(th2 + th3 + th4)],
                    [np.sin(th1)],
                    [-np.cos(th1)],
                    [0]])

    J3 = np.array([[D5*(np.cos(th1)*np.cos(th5) + np.cos(th2+th3+th4)*np.sin(th1)*np.sin(th5)) + D3*np.cos(th1) + D4*np.cos(th1) - A3*np.cos(th2+th3)*np.sin(th1) - A2*np.cos(th2)*np.sin(th1) - D5*np.sin(th2+th3+th4)*np.sin(th1)],
                    [D5*(np.cos(th5)*np.sin(th1) - np.cos(th2+th3+th4)*np.cos(th1)*np.sin(th5)) + D3*np.sin(th1) + D4*np.sin(th1) + A3*np.cos(th2+th3)*np.cos(th1) + A2*np.cos(th1)*np.cos(th2) - D5*np.sin(th2+th3+th4)*np.cos(th1)],
                    [0],
                    [0],
                    [0],
                    [1]])
    
    J4 = np.array([[D5*(np.cos(th1)*np.cos(th5) + np.cos(th2+th3+th4)*np.sin(th1)*np.sin(th5)) + D3*np.cos(th1) + D4*np.cos(th1) - A3*np.cos(th2+th3)*np.sin(th1) - A2*np.cos(th2)*np.sin(th1) - D5*np.sin(th2+th3+th4)*np.sin(th1)],
                    [D5*(np.cos(th5)*np.sin(th1) - np.cos(th2+th3+th4)*np.cos(th1)*np.sin(th5)) + D3*np.sin(th1) + D4*np.sin(th1) + A3*np.cos(th2+th3)*np.cos(th1) + A2*np.cos(th1)*np.cos(th2) - D5*np.sin(th2+th3+th4)*np.cos(th1)],
                    [0],
                    [0],
                    [0],
                    [1]])

    J5 = np.array([[D5*(np.cos(th1)*np.cos(th5) + np.cos(th2+th3+th4)*np.sin(th1)*np.sin(th5)) + D3*np.cos(th1) + D4*np.cos(th1) - A3*np.cos(th2+th3)*np.sin(th1) - A2*np.cos(th2)*np.sin(th1) - D5*np.sin(th2+th3+th4)*np.sin(th1)],
                    [D5*(np.cos(th5)*np.sin(th1) - np.cos(th2+th3+th4)*np.cos(th1)*np.sin(th5)) + D3*np.sin(th1) + D4*np.sin(th1) + A3*np.cos(th2+th3)*np.cos(th1) + A2*np.cos(th1)*np.cos(th2) - D5*np.sin(th2+th3+th4)*np.cos(th1)],
                    [0],
                    [0],
                    [0],
                    [1]])

    J6 = np.array([[D5*(np.cos(th1)*np.cos(th5) + np.cos(th2+th3+th4)*np.sin(th1)*np.sin(th5)) + D3*np.cos(th1) + D4*np.cos(th1) - A3*np.cos(th2+th3)*np.sin(th1) - A2*np.cos(th2)*np.sin(th1) - D5*np.sin(th2+th3+th4)*np.sin(th1)],
                    [D5*(np.cos(th5)*np.sin(th1) - np.cos(th2+th3+th4)*np.cos(th1)*np.sin(th5)) + D3*np.sin(th1) + D4*np.sin(th1) + A3*np.cos(th2+th3)*np.cos(th1) + A2*np.cos(th1)*np.cos(th2) - D5*np.sin(th2+th3+th4)*np.cos(th1)],
                    [0],
                    [0],
                    [0],
                    [1]])


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
    threshold = 0.0011
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

    while not (current_aperture[0]>value-threshold and current_aperture[0]<value+threshold):
        print("waiting for gripper")
        

def moveTo(xef, phief):
    pub = rospy.Publisher('/trajectory_controller/command', JointTrajectory, queue_size=10)
    #rospy.Subscriber('/trajectory_controller/state', String, queue_size=10)
    
    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']

    rate = rospy.Rate(10)

    TH0 = current_pos
    xe0, Re = ur5Direct(TH0)
    phie0 = np.transpose(rotm2eul(Re))

    xef[1] -= 0.024
    xef = np.transpose(xef)
    phief = np.transpose(phief)

    xe = lambda t: np.dot(t,xef) + np.dot((1-t),xe0)
    phie = lambda t: np.dot(t,phief) + np.dot((1-t),phie0)

    x = xe(1)
    phi = phie(1)
    phi = np.transpose(phi)
    Th = ur5Inverse(x, rot.from_euler('ZYX', [phi[0], phi[1], phi[2]]).as_dcm()) # A seconda da che versioni di scipy: as_dcm() o as_matrix()
    while not rospy.is_shutdown():

        threshold = 0.008
        if (current_pos[0]>Th[6][0]-threshold and current_pos[0]<Th[6][0]+threshold) and (current_pos[1]>Th[6][1]-threshold and current_pos[1]<Th[6][1]+threshold) and (current_pos[2]>Th[6][2]-threshold and current_pos[2] < Th[6][2]+threshold) and (current_pos[3]>Th[6][3]-threshold and current_pos[3] < Th[6][3]+threshold) and (current_pos[4]>Th[6][4]-threshold and current_pos[4] < Th[6][4]+threshold) and (current_pos[5]>Th[6][5]-threshold and current_pos[5] < Th[6][5]+threshold):
            break

        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()

        #pts.positions = [0, -1.5, 1.0, 0, 0, 0]
        pts.positions = [Th[6][0], Th[6][1], Th[6][2], Th[6][3], Th[6][4], Th[6][5]]
        pts.time_from_start = rospy.Duration(0.5)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)

def jointState(msg):
    # ------
    # JointState structure
    # name: -elbow_joint -robotiq_85_left_knuckle_joint -shoulder_lift_joint -shoulder_pan_joint -wrist_1_joint -wrist_2_joint -wrist_3_joint

    #['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
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
    
    #lego_1: 0.53

    #Final position of end effector
    xef = np.array([0.4, 0.14, 0.5])
    #Final orientation of end effector
    phief = np.array([np.pi/2, np.pi, 0]) #[np.pi, np.pi, 0] for vertical gripper, [np.pi/2, np.pi, 0] for horizontal gripper]

    gripper_client(0.0)

    #Calculate joint angle matrix
    Th = moveTo(xef, phief)    

    xef = np.array([0.4, 0.14, 0.42])
    Th = moveTo(xef, phief)  

    gripper_client(0.53)
    time.sleep(3)

    xef = np.array([0.3, -0.6, 0.6])
    Th = moveTo(xef, phief)  

    gripper_client(0.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
