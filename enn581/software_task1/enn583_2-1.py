import roboticstoolbox as rtb
from swift import Swift
import spatialmath as sm
import spatialgeometry as sg
import numpy as np
import math
import matplotlib.pyplot as plt

ets = rtb.ETS([
    rtb.ET.Rz(), # q0
    rtb.ET.tz(400),
    rtb.ET.tx(180),
    rtb.ET.Rx(90, 'deg'), 
    rtb.ET.Rz(), # q1
    rtb.ET.tx(600),
    rtb.ET.Rz(), # q2
    rtb.ET.tx(120),
    rtb.ET.Rx(90, 'deg'),
    rtb.ET.Rz(), # q3
    rtb.ET.tz(620),
    rtb.ET.Rx(-90, 'deg'),
    rtb.ET.Rz(), # q4,
    rtb.ET.Rx(90, 'deg'),
    rtb.ET.Rz(), # q5
    rtb.ET.tz(115) # end-eff
    ])
robot = rtb.Robot(ets)

qlim = np.array([
    [math.radians(-155), math.radians(155)],
    [-math.pi, math.radians(65)],
    [math.radians(-15),math.radians(158)],
    [math.radians(-350),math.radians(350)],
    [math.radians(-130),math.radians(130)],
    [math.radians(-350),math.radians(350)]], dtype=float).transpose()


def robotFK(q):
    '''
    Get and print the transform, T, of the end effector with respect to the base

        Parameters:
            q (list<int>): joint angles (rad) for the robot (len() must match no. joints)
            
        Returns:
            T (sm.SE3): The transformation matrix representing the pose of the end-effector (wrt base)
    '''
    robot
    T = robot.fkine(q)
    print(T)
    return T

def get_next_workspace_config(q, qlim, dTheta, joint_idx):
    '''
    Find the next valid joint config for workspace search
    ** Recursive **
    
        Parameters:
            q (list<int>): joint angles (rad) for the robot (len() must match no. joints)
            qlim (np.array): 2D (2, n) numpy array of min/max angle limits (rad) of joints
            dTheta (np.array): Angular resolution (in radians) of each joint (1,n)
            joint_idx (int): Index of joint to be updated
    '''
    if joint_idx >= (len(q) - 1): # last joint gives us no new positions
        pass
    else:
        q[joint_idx] += dTheta[joint_idx]
        if q[joint_idx] > qlim[1,joint_idx]:
            q[joint_idx] = qlim[0, joint_idx]
            q = get_next_workspace_config(q, qlim, dTheta, joint_idx+1)
    return q

def robotWorkspace(qlimits):
    '''
    Write a csv file with the x,y,z coordinates within the robot's workspace. \
    Joints have a resolution (degrees) of: 
        [1 15 15 15 15 N/A]
        
        Parameters:
            qlimits (np.array): 2D numpy array of min/max angle limits (rad) of joints
            
        Returns:
            numpy array of coordinates the robot can reach (1 degree resolution)
    '''
    # number_points = 10
    q_start = np.asarray([lim for lim in qlimits[0,:]])
    q = q_start.copy()
    # dTheta = np.array([(qlim[1]-qlim[0]) * (1/number_points) for qlim in qlimits.transpose()])
    dTheta = [math.radians(5), math.radians(20), math.radians(20), math.radians(20), math.radians(20), math.radians(qlimits[1, 5])]
    
    coords = []
    while True:
        pos = robotFK(q).t
        coords.append((pos[0], pos[1], pos[2]))
        q = get_next_workspace_config(q, qlimits, dTheta, 0)
        if np.allclose(q, q_start, atol=math.radians(0.5)):
            break
    
    with open('workspace.csv', 'w') as csv:
        for coord in coords:
            csv.write(f'{coord[0]}, {coord[1]}, {coord[2]}\n')
    
def robotIK(T):
    '''
    Get and print the joint positions (in rtb.IKSolution form), q, for a given end effector pose
    
        Parameters:
            T (sm.SE3): Desired end effector pose
            
        Returns:
            q (rtb.IKSolution): Inverse kinematics solution
    '''
    q = robot.ikine_NR(T)
    print(q)
    return q

def get_joint_pose(idx: int):
    T = sm.SE3()
    for i in range(idx+1):
        T @= robot.links[i].A()
    print(T)
    return T

if __name__ == '__main__':
    robotWorkspace(qlim)
    T = sm.SE3.Trans(2,2,2) @ sm.SE3.Rx(90, 'deg')
    q = robotIK(T).q
    robotFK(q)
    