# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 15:24:40 2024

@author: ajvan
"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.io as io

#basic helper functions
deg2rad = lambda x: x*np.pi/180
rad2deg = lambda x: x*180/np.pi

def wrap_to_pi(x):
    while x < -np.pi:
        x += 2*np.pi
    while x > np.pi:
        x -= 2*np.pi
    return x
'''
Create a plot in matplotlib when initialising the sim and return its handle 
to continue updating it throughout.
'''
n_landmarks = 10
data = io.loadmat('slam_data.mat')


def create_plot(mu, Sigma, lmarks):
    handle = plt.plot([mu[1,0]], [mu[2,0]])
    plt.axis([-2, 5, -2, 5])
    plt.show()
    return handle

#Update the plot at each stage of the simulation
def update_plot(mu, Sigma):
    return

def odom_jacobians(d, th):
    J1 = np.matrix([[1, 0, -d*np.sin(th)],
                    [0, 1, d*np.cos(th)],
                    [0, 0, 1]])
    
    J2 = np.matrix([[np.cos(th), 0],
                    [np.sin(th), 0],
                    [0, 1]])
    return J1, J2

def z_jacobians(landmark_pos, robot_pos):
    xl = landmark_pos[0,0]
    yl = landmark_pos[1,0]
    xr = robot_pos[0,0]
    yr = robot_pos[1,0]
    r = np.sqrt((xr-xl)**2 + (yr - yl)**2)
    
    Gz = np.matrix([[(xl-xr)/r, (yl-yr)/r],
          [-(yl-yr)/(r**2), (xl-xr)/(r**2)]])
    Gv = np.matrix([[-(xl-xr)/r, -(yl-yr)/r, 0],
        [(yl-yr)/(r**2), -(xl-xr)/(r**2), -1]])
    return Gz, Gv

def l_jacobian(r, b, th):
    L = [[np.cos(th+b), -r*np.sin(th+b)],
         [np.sin(th+b), r*np.cos(th+b)]]
    return np.matrix(L)


def predict_z(mu, landmark_id):
    lidx = 3 + landmark_id*2
    xl = mu[lidx, 0]
    yl= mu[lidx+1, 0]
    xr = mu[0,0]
    yr = mu[1,0]
    thr = mu[2,0]
    r = np.sqrt((xr-xl)**2 + (yr - yl)**2)
    b = np.arctan2(yl-yr, xl-xr) - thr
    return np.matrix([[r], [wrap_to_pi(b)]])


def get_odom(step):
    
    return data['odom'][step]

def sense_landmarks(step):
    idx = step*n_landmarks
    return data['sensor'][range(idx,idx+10)]

def init_landmarks(z, Q, mu, Sigma):
    x = mu[0,0]
    y = mu[1,0]
    th = mu[2,0]
    
    for r, b in z:
        n = len(mu)
        xl = x + r*np.cos(th+b)
        yl = y + r*np.sin(th+b)
        mu = np.vstack((mu, np.matrix([[xl], [yl]])))
        L = l_jacobian(r, b, th)
        lSigma = L*Q*L.transpose()
        Sigma = np.vstack(
            (np.hstack((Sigma, np.zeros((n, 2)))),
             np.hstack((np.zeros((2,n)), lSigma)))
            )
        
    return mu, Sigma

def predict_step(mu, Sigma, d, dth, R):
    th = mu[2,0]
    updated = np.matrix([[mu[0,0] + d*np.cos(th)],
                         [mu[1,0] + d*np.sin(th)],
                         [th + dth]])
    J1, J2 = odom_jacobians(d, th)
    s = np.size(Sigma.diagonal())
    if s == 3:
        Sigma = J1*Sigma*J1.transpose() + J2*R*J2.transpose()
    else:
        J1_padded = np.matrix(np.zeros((s,s)))
        J1_padded[0:3, 0:3] = J1
        J2_padded = np.matrix(np.zeros((s, 2)))
        J2_padded[0:3, :] = J2
        Sigma = J1_padded*Sigma*J1_padded.transpose() + J2_padded*R*J2_padded.transpose()
        updated = np.vstack((updated, mu[3:, 0]))
    return updated, Sigma

def update_step(landmark_id, z, mu, Sigma, Q):
    n = len(mu) - 3
    lidx = 3 + landmark_id*2
    l = mu[[lidx, lidx+1], 0]
    Giz, Giv = z_jacobians(l,mu[0:3, 0])
    G = np.matrix(np.zeros((2,3+n)))
    G[:, [lidx,lidx+1]] = Giz 
    G[:,0:3] = Giv
    zp = predict_z(mu,landmark_id)          
    K1 = Sigma * G.transpose()
    K2 = G * Sigma * G.transpose() + Q
    K = K1 * np.linalg.inv(K2)
    r = z - zp
    r[1,0] = wrap_to_pi(r[1,0])
    err = K * r
    mu = mu + err
    Sigma = (np.matrix(np.eye(3+n)) - K*G)*Sigma
    Sigma = 0.5*(Sigma+Sigma.transpose())
    return mu, Sigma


if __name__ == "__main__":
    #number of steps to run the simulation for
    nsteps = 100
    
    #Robot initial conditions and uncertainty
    mu = np.matrix([0, 0, deg2rad(0)]).transpose()
    Sigma = np.matrix(np.diag([0.1, 0.1, deg2rad(0.1)])**2)
    
    #Uncertainty in odometry (R) and sensor (Q)
    R = np.matrix(np.diag((0.5, deg2rad(50)))**2)
    Q = np.matrix(np.diag((0.5, deg2rad(5)))**2)
    
    true_landmark_positions = data['map']
    true_robot_pose = data['xr']
    
    #Create plot with initial conditions
    plt_fig = create_plot(mu, Sigma, true_landmark_positions)
    
    #Run simulation for specified number of steps
    for step in range(nsteps):
        
        d, dth = get_odom(step)
        
        mu, Sigma = predict_step(mu, Sigma, d, dth, R)
        
        z = sense_landmarks(step)
        
        if step == 0:
            mu, Sigma = init_landmarks(z, Q, mu, Sigma)
            continue
         
        for landmark_id, (r, b) in enumerate(z):
            zi = np.matrix(
                [[r],
                 [b]]
                )
            mu, Sigma = update_step(landmark_id, zi, mu, Sigma, Q)
        
        update_plot(mu, Sigma)
        
        