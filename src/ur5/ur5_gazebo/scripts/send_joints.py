#!/usr/bin/python3

from std_msgs.msg import Header
from std_msgs.msg import String
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

#Contains current joint angles -> updated every time
#['shoulder_pan_joint', 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint','wrist_3_joint']
current_pos = [0, -1.5, 1.0, 0.0, 0.0, 0]
# Contains gripper aperture -> 0.0 to 0.8
current_aperture = [0]

def ur5Direct(Th):
    A = [0, -0.425, -0.3922, 0, 0, 0]
    D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]

    T10f = lambda th1: np.array([[np.cos(th1), -np.sin(th1), 0, 0],
                        [np.sin(th1), np.cos(th1), 0, 0],
                        [0, 0, 1, D[0]],
                        [0, 0, 0, 1]])

    T21f = lambda th2: np.array([[np.cos(th2), -np.sin(th2), 0, 0],
                        [0, 0, -1, 0],
                        [np.sin(th2), np.cos(th2), 0, 0],
                        [0, 0, 0, 1]])

    T32f = lambda th3: np.array([[np.cos(th3), -np.sin(th3), 0, A[1]],
                        [np.sin(th3), np.cos(th3), 0, 0],
                        [0, 0, 1, D[2]],
                        [0, 0, 0, 1]])

    T43f = lambda th4: np.array([[np.cos(th4), -np.sin(th4), 0, A[2]],
                        [np.sin(th4), np.cos(th4), 0, 0],
                        [0, 0, 1, D[3]],
                        [0, 0, 0, 1]])

    T54f = lambda th5: np.array([[np.cos(th5), -np.sin(th5), 0, 0],
                        [0, 0, -1, -D[4]],
                        [np.sin(th5), np.cos(th5), 0, 0],
                        [0, 0, 0, 1]])

    T65f = lambda th6: np.array([[np.cos(th6), -np.sin(th6), 0, 0],
                        [0, 0, 1, D[5]],
                        [-np.sin(th6), -np.cos(th6), 0, 0],
                        [0, 0, 0, 1]])
    
    T10m = T10f(Th[0])
    T21m = T21f(Th[1])
    T32m = T32f(Th[2])
    T43m = T43f(Th[3])
    T54m = T54f(Th[4])
    T65m = T65f(Th[5])

    T06 = np.dot(np.dot(np.dot(T10m,T21m),np.dot(T32m,T43m)),np.dot(T54m,T65m))

    pe = T06[0:3,3]
    Re = T06[0:3, 0:3]

    return pe,Re

# Inverse kinematic for all 6 joints
def ur5Inverse(p60, R60):
    A = [0, -0.425, -0.3922, 0, 0, 0]
    D = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]
    
    T60 = np.array([[R60[0][0], R60[0][1], R60[0][2], p60[0]],
            [R60[1][0], R60[1][1], R60[1][2], p60[1]],
            [R60[2][0], R60[2][1], R60[2][2], p60[2]],
            [0, 0, 0, 1]])

    T10f = lambda th1: np.array([[np.cos(th1), -np.sin(th1), 0, 0],
                        [np.sin(th1), np.cos(th1), 0, 0],
                        [0, 0, 1, D[0]],
                        [0, 0, 0, 1]])
    
    T21f = lambda th2: np.array([[np.cos(th2), -np.sin(th2), 0, 0],
                        [0, 0, -1, 0],
                        [np.sin(th2), np.cos(th2), 0, 0],
                        [0, 0, 0, 1]])

    T32f = lambda th3: np.array([[np.cos(th3), -np.sin(th3), 0, A[1]],
                        [np.sin(th3), np.cos(th3), 0, 0],
                        [0, 0, 1, D[2]],
                        [0, 0, 0, 1]])

    T43f = lambda th4: np.array([[np.cos(th4), -np.sin(th4), 0, A[2]],
                        [np.sin(th4), np.cos(th4), 0, 0],
                        [0, 0, 1, D[3]],
                        [0, 0, 0, 1]])

    T54f = lambda th5: np.array([[np.cos(th5), -np.sin(th5), 0, 0],
                        [0, 0, -1, -D[4]],
                        [np.sin(th5), np.cos(th5), 0, 0],
                        [0, 0, 0, 1]])

    T65f = lambda th6: np.array([[np.cos(th6), -np.sin(th6), 0, 0],
                        [0, 0, 1, D[5]],
                        [-np.sin(th6), -np.cos(th6), 0, 0],
                        [0, 0, 0, 1]])
    
    # Finding Th1
    x = [0, 0, -D[5], 1]
    x = np.transpose(x)
    p50 = T60*x
    p50 = np.dot(T60, x)

    th1_1= np.real(np.arctan2(p50[1], p50[0]) + np.arccos(D[3] / np.hypot(p50[1], p50[0]))) + (np.pi/2)
    th1_2= np.real(np.arctan2(p50[1], p50[0]) - np.arccos(D[3] / np.hypot(p50[1], p50[0]))) + (np.pi/2)

    # Finding Th5
    th5_1= +np.real(np.arccos((p60[0]*np.sin(th1_1) - p60[1]*np.cos(th1_1)-D[3]) / D[5]))
    th5_2= -np.real(np.arccos((p60[0]*np.sin(th1_1) - p60[1]*np.cos(th1_1)-D[3]) / D[5]))
    th5_3= +np.real(np.arccos((p60[0]*np.sin(th1_2) - p60[1]*np.cos(th1_2)-D[3]) / D[5]))
    th5_4= -np.real(np.arccos((p60[0]*np.sin(th1_2) - p60[1]*np.cos(th1_2)-D[3]) / D[5]))

    T06 = np.linalg.inv(T60)
    Xhat = T06[0:3,0]
    Yhat = T06[0:3,1]

    th6_1 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_1)+Yhat[1]*np.cos(th1_1)))/np.sin(th5_1), ((Xhat[0]*np.sin(th1_1)-Yhat[0]*np.cos(th1_1)))/np.sin(th5_1)))
    th6_2 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_1)+Yhat[1]*np.cos(th1_1)))/np.sin(th5_2), ((Xhat[0]*np.sin(th1_1)-Yhat[0]*np.cos(th1_1)))/np.sin(th5_2)))
    th6_3 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_2)+Yhat[1]*np.cos(th1_2)))/np.sin(th5_3), ((Xhat[0]*np.sin(th1_2)-Yhat[0]*np.cos(th1_2)))/np.sin(th5_3)))
    th6_4 = np.real(np.arctan2(((-Xhat[1]*np.sin(th1_2)+Yhat[1]*np.cos(th1_2)))/np.sin(th5_4), ((Xhat[0]*np.sin(th1_2)-Yhat[0]*np.cos(th1_2)))/np.sin(th5_4)))
    
    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_1)),T60),np.dot(np.linalg.inv(T65f(th6_1)),np.linalg.inv(T54f(th5_1))))
    p41_1 = T41m[0:3, 3]
    p41xz_1 = np.hypot(p41_1[0], p41_1[2])

    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_1)),T60),np.dot(np.linalg.inv(T65f(th6_2)),np.linalg.inv(T54f(th5_2))))
    p41_2 = T41m[0:3, 3]
    p41xz_2 = np.hypot(p41_2[0], p41_2[2])

    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_2)),T60),np.dot(np.linalg.inv(T65f(th6_3)),np.linalg.inv(T54f(th5_3))))
    p41_3 = T41m[0:3, 3]
    p41xz_3 = np.hypot(p41_3[0], p41_3[2])

    T41m = np.dot(np.dot(np.linalg.inv(T10f(th1_2)),T60),np.dot(np.linalg.inv(T65f(th6_4)),np.linalg.inv(T54f(th5_4))))
    p41_4 = T41m[0:3, 3]
    p41xz_4 = np.hypot(p41_4[0], p41_4[2])
    
    th3_1 = np.real(np.emath.arccos((p41xz_1**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))
    th3_2 = np.real(np.emath.arccos((p41xz_2**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))
    th3_3 = np.real(np.emath.arccos((p41xz_3**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))
    th3_4 = np.real(np.emath.arccos((p41xz_4**2-A[1]**2-A[2]**2)/(2*A[1]*A[2])))

    th3_5 = -th3_1
    th3_6 = -th3_2
    th3_7 = -th3_3
    th3_8 = -th3_4

    th2_1 = np.real(np.arctan2(-p41_1[2], -p41_1[0])-np.arcsin((-A[2]*np.sin(th3_1))/p41xz_1))
    th2_2 = np.real(np.arctan2(-p41_2[2], -p41_2[0])-np.arcsin((-A[2]*np.sin(th3_2))/p41xz_2))
    th2_3 = np.real(np.arctan2(-p41_3[2], -p41_3[0])-np.arcsin((-A[2]*np.sin(th3_3))/p41xz_3))
    th2_4 = np.real(np.arctan2(-p41_4[2], -p41_4[0])-np.arcsin((-A[2]*np.sin(th3_4))/p41xz_4))

    th2_5 = np.real(np.arctan2(-p41_1[2], -p41_1[0])-np.arcsin((A[2]*np.sin(th3_1))/p41xz_1))
    th2_6 = np.real(np.arctan2(-p41_2[2], -p41_2[0])-np.arcsin((A[2]*np.sin(th3_2))/p41xz_2))
    th2_7 = np.real(np.arctan2(-p41_3[2], -p41_3[0])-np.arcsin((A[2]*np.sin(th3_3))/p41xz_3))
    th2_8 = np.real(np.arctan2(-p41_4[2], -p41_4[0])-np.arcsin((A[2]*np.sin(th3_4))/p41xz_4))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_1)),np.linalg.inv(T21f(th2_1))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_1)),np.linalg.inv(T54f(th5_1))))
    Xhat43 = T43m[0:3,0]
    th4_1 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_2)),np.linalg.inv(T21f(th2_2))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_2)),np.linalg.inv(T54f(th5_2))))
    Xhat43 = T43m[0:3,0]
    th4_2 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_3)),np.linalg.inv(T21f(th2_3))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_3)),np.linalg.inv(T54f(th5_3))))
    Xhat43 = T43m[0:3,0]
    th4_3 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_4)),np.linalg.inv(T21f(th2_4))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_4)),np.linalg.inv(T54f(th5_4))))
    Xhat43 = T43m[0:3,0]
    th4_4 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_5)),np.linalg.inv(T21f(th2_5))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_1)),np.linalg.inv(T54f(th5_1))))
    Xhat43 = T43m[0:3,0]
    th4_5 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_6)),np.linalg.inv(T21f(th2_6))),np.dot(np.linalg.inv(T10f(th1_1)),T60)),np.dot(np.linalg.inv(T65f(th6_2)),np.linalg.inv(T54f(th5_2))))
    Xhat43 = T43m[0:3,0]
    th4_6 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_7)),np.linalg.inv(T21f(th2_7))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_3)),np.linalg.inv(T54f(th5_3))))
    Xhat43 = T43m[0:3,0]
    th4_7 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))

    T43m = np.dot(np.dot(np.dot(np.linalg.inv(T32f(th3_8)),np.linalg.inv(T21f(th2_8))),np.dot(np.linalg.inv(T10f(th1_2)),T60)),np.dot(np.linalg.inv(T65f(th6_4)),np.linalg.inv(T54f(th5_4))))
    Xhat43 = T43m[0:3,0]
    th4_8 = np.real(np.arctan2(Xhat43[1],Xhat43[0]))
    
    
    Th = [[th1_1, th2_1, th3_1, th4_1, th5_1, th6_1],
          [th1_1, th2_2, th3_2, th4_2, th5_2, th6_2],
          [th1_2, th2_3, th3_3, th4_3, th5_3, th6_3],
          [th1_2, th2_4, th3_4, th4_4, th5_4, th6_4],
          [th1_1, th2_5, th3_5, th4_5, th5_1, th6_1],
          [th1_1, th2_6, th3_6, th4_6, th5_2, th6_2],
          [th1_2, th2_7, th3_7, th4_7, th5_3, th6_3],
          [th1_2, th2_8, th3_8, th4_8, th5_4, th6_4]]

    print("Th:")
    print(Th)

    return Th

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

    xef[1] -= 0.02
    xef = np.transpose(xef)
    phief = np.transpose(phief)

    xe = lambda t: np.dot(t,xef) + np.dot((1-t),xe0)
    phie = lambda t: np.dot(t,phief) + np.dot((1-t),phie0)

    x = xe(1)
    phi = phie(1)
    phi = np.transpose(phi)
    Th = ur5Inverse(x, rot.from_euler('ZYX', [phi[0], phi[1], phi[2]]).as_dcm())
    while not rospy.is_shutdown():

        threshold = 0.003
        if (current_pos[0]>Th[6][0]-threshold and current_pos[0]<Th[6][0]+threshold) and (current_pos[1]>Th[6][1]-threshold and current_pos[1]<Th[6][1]+threshold) and (current_pos[2]>Th[6][2]-threshold and current_pos[2] < Th[6][2]+threshold) and (current_pos[3]>Th[6][3]-threshold and current_pos[3] < Th[6][3]+threshold) and (current_pos[4]>Th[6][4]-threshold and current_pos[4] < Th[6][4]+threshold) and (current_pos[5]>Th[6][5]-threshold and current_pos[5] < Th[6][5]+threshold):
            break

        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()

        #pts.positions = [0, -1.5, 1.0, 0, 0, 0]
        pts.positions = [Th[6][0], Th[6][1], Th[6][2], Th[6][3], Th[6][4], Th[6][5]]
        pts.time_from_start = rospy.Duration(1.0)

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
    
    #Final position of end effector
    xef = np.array([0.60, -0.43, 0.5])
    #Final orientation of end effector
    phief = np.array([np.pi/2, np.pi, 0]) #[np.pi, np.pi, 0] for vertical gripper, [np.pi/2, np.pi, 0] for horizontal gripper

    gripper_client(0.0)

    #Calculate joint angle matrix
    Th = moveTo(xef, phief)    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
