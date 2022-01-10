import numpy as np
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