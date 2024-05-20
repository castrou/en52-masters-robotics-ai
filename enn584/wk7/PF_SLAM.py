# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 15:24:40 2024

@author: ajvan
"""

import matplotlib.pyplot as plt
import numpy as np
import scipy.io as io
from particle_filter import ParticleFilter

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


def plot_cov(x, S, nS):
    vals, vecs = np.linalg.eig(S)
    a = nS * np.linspace(0, 2*np.pi, 100)
    y = np.matrix(np.vstack((np.cos(a), np.sin(a))))
    # el = vecs * np.sqrt(vals) @ np.linalg.inv(vecs)
    return xvals, yvals

def create_plot(particles, lmarks):
    handle = plt.plot([mu[1,0]], [mu[2,0]])
    plt.axis([-2, 5, -2, 5])
    plt.show()
    return handle

#Update the plot at each stage of the simulation
def update_plot(mu, Sigma):
    return

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



if __name__ == "__main__":
    #number of steps to run the simulation for
    nsteps = 100
    
    #Initialise particle filter
    mu = [0, 0, 0]
    Sigma = np.array([0.1, 0.1, deg2rad(0.1)])**2
    pf = ParticleFilter(mu, Sigma, n_particles=10)
    
    #Uncertainty in odometry (R) and sensor (Q)
    R = np.matrix(np.diag((0.5, deg2rad(50)))**2)
    Q = np.matrix(np.diag((0.5, deg2rad(5)))**2)
    
    true_landmark_positions = data['map']
    true_robot_pose = data['xr']
    
    #Create plot with initial conditions
    plt_fig = create_plot(pf.particles, true_landmark_positions)
    
    #Run simulation for specified number of steps
    for step in range(nsteps):
        
        d, dth = get_odom(step)
    
        estimated_pose = pf.predict_all(d, dth, R)
        
        z = sense_landmarks(step)
        
        if step == 0:
            global_map = pf.init_landmarks(z, Q, mu, Sigma)
            continue
         
        global_map = pf.update(z, global_map)
        
        update_plot(pf_filter, global_map)
        
        