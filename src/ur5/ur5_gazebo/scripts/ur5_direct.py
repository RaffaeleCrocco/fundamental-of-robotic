# Direct kinematic function
import numpy as np

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