import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
import roboticstoolbox as rtb

from sympy import *

R = 1
D = 1

theta = Symbol("theta", real=True)
# W1 = np.array([lambda x: np.cos(x) / R,
#                lambda y: np.sin(y) / R,
#                 -D / R])

W1 = Matrix([cos(theta)/R, sin(theta)/R, -D/R])
W2 = Matrix([(sqrt(3)*sin(theta) - cos(theta))/(2*R), 
               (-sin(theta) - sqrt(3)*cos(theta))/(2*R),
               -D/2*R])
W3 = Matrix([(-cos(theta) - sqrt(3)*sin(theta))/(2*R), 
               (sqrt(3)*cos(theta) -sin(theta))/(2*R),
               -D/2*R])

def get_velocity_vector(wheel, angular_accel, th):
    if wheel == 'phi1':
        W = W1
    elif wheel == 'phi2':
        W = W2
    elif wheel == 'phi3':
        W = W3
    expr = W.pinv() * angular_accel
    vel_vec = expr.subs(theta, th)
    return vel_vec

def plot_trajectory(angular_accel, duration):
    wheels = ['phi1', 'phi2', 'phi3']
    x = 0
    y = 0
    th = 0
    traj = np.empty((0, 3))
    for t in range(0, duration):
        xd = 0
        yd = 0
        thd = 0
        for wheel, ang_acc in zip(wheels, angular_accel):
            vel_vec = get_velocity_vector(wheel, ang_acc, th)
            xd += N(vel_vec[0])
            yd += N(vel_vec[1])
            thd += N(vel_vec[2])
        y += float(yd * 0.001)
        x += float(xd * 0.001)
        th += float(thd * 0.001)
        traj = np.vstack((traj, np.array([x, y, th])))
        
    return traj


if __name__ == '__main__':
    phi = [1, 1, 1]
    
    traj = plot_trajectory(phi, 100)
    
    plt.arrow(traj[:,0], traj[:,1], np.cos(traj[:,2]), np.sin(traj[:,2]))
    
