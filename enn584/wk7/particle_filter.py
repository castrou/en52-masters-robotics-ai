# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 14:18:45 2024

@author: ajvan
"""

import numpy as np

class ParticleFilter:
    def __init__(self, mu, Sigma, n_particles=25, default_weight=1, add_noise=True):
        self.default_weight = default_weight
        self.n_particles = n_particles
        self.n_resample = int(self.n_particles/10)
        self.particles = np.zeros([3, self.n_particles])
        self.alphas = np.ones(self.n_particles) * (1/self.n_particles)
        self.ones = np.ones_like(self.alphas)
        self.add_noise = add_noise
        if self.add_noise:
            self.particles = np.vstack((np.random.normal(mu[0], Sigma[0], self.n_particles),
                                        np.random.normal(mu[1], Sigma[1], self.n_particles),
                                        np.random.normal(mu[2], Sigma[2], self.n_particles)
                                        ))
        else:
             self.particles[0, :] = mu[0]
             self.particles[1, :] = mu[1]
             self.particles[2, :] = mu[2]
        self.local_maps = np.zeros((self.n_particles, 2, 10))
        self.weights = np.ones((1, self.n_particles))
        self.est_pose = np.array([0,0,0], dtype=float)

    def predict_all(self, d, dth, R):
        # add noise
        if self.add_noise:
            noisy_d = d + np.random.normal(0, R[0,0], self.n_particles)
            noisy_dth = dth + np.random.normal(0, R[1,1], self.n_particles)
        else:
            noisy_d = d
            noisy_dth = dth * self.ones

        # use motion model
        theta = self.particles[-1,:]
        self.particles += np.vstack([noisy_d * np.cos(theta),
                                         noisy_d * np.sin(theta),
                                         noisy_dth])
        
        #estimated position of robot is weighted average of particle locations
        pose = np.array([0,0,0], dtype=float)
        total_weight = 0
        for idx in range(self.n_particles):
            pose += self.weights[idx] * self.particles[:, idx]
            total_weight += self.weights[idx]
            
        pose = pose / total_weight
        self.est_pose = pose
        return pose

    def update(self, z, global_map):
        '''
        Inputs:
            z: sensor measurements, (r,b) to each landmark
            global_map: best estimate of robot pose and landmark positions before update
        Outputs:
            self.weights is updated
            global_map: updated and returns
        '''
        
        self.update_weights(z, global_map)
        
        #then update global map weighted avg
        total_weight = 0
        for idx in range(self.n_particles):
            global_map += self.weights[idx] * self.local_maps[idx,:,:]
            total_weight += self.weights[idx]
        global_map = global_map / total_weight
        return global_map
        
    def update_weights(self, z, global_map):
        '''
        As part of the update step in PF SLAM, we need to update our weights
        for each particle according to how well new measurements line up with
        their positions
        
        To be called in self.update() method
        '''
        residuals = np.zeros((self.n_particles,landmark_id))
        for landmark_id, (r, b) in enumerate(z):
            #update global map similarly
            global_map[:,landmark_id] = self.est_pose[:2] + np.vstack([r*np.cos(b + self.est_pose[2]),
                                            r*np.sin(b + self.est_pose[2])])
            
            #calculate position of landmark based on particle and sensor
            #update local maps accordingly
            for particle_id, particle in enumerate(self.particles):
                rel_landmark_pos = particle[:2] + np.vstack([r*np.cos(b + particle[2]),
                                                    r*np.sin(b + particle[2])])
                self.local_maps[particle_id, :, landmark_id] = rel_landmark_pos
                residuals[particle_id,landmark_id] = np.sqrt((global_map[0,landmark_id] - rel_landmark_pos[0])**2 + 
                                           (global_map[1,landmark_id] - rel_landmark_pos[1])**2)
                
        err = np.zeros(self.n_particles)
        for i, residual in enumerate(residuals):
            err[i] = np.sum(residual) / 10
            
        mean_err = np.mean(err)
        for i, error in enumerate(err):
            self.weights[i] += mean_err / error
    
    def init_landmarks(self, z):
        '''
        Inputs:
            self: look at the initialization method to see what properties this
                class has
            z: sensor measurements of range and bearing to all landmarks
        Output:
            Global maps of landmark positions (should be a numpy array of size (2,2xn))
            save local maps for each particle to self.map (should be numpy array of size (n, 2, 2xn))
        '''
        #look at predict_all to see how the robot's position is estimated from
        #each particle. How can you use this idea to estimate landmark positions?
            
        global_map = np.zeros((2,10))
        for idx in range(self.n_particles):
            for landmark_id, (r, b) in enumerate(z):
                #calculate position of landmark based on particle and sensor
                #update local maps accordingly
                for particle_id, particle in enumerate(self.particles):
                    rel_landmark_pos = particle[:2] + np.vstack([r*np.cos(b + particle[2]),
                                                        r*np.sin(b + particle[2])])
                    self.local_maps[particle_id, :, landmark_id] = rel_landmark_pos
                    
                #then update global map similarly
                total_weight = 0
                for idx in range(self.n_particles):
                    global_map[:,landmark_id] += self.weights[idx] * self.local_maps[idx,:,landmark_id]
                    total_weight += self.weights[idx]
                global_map[:,landmark_id] = global_map / total_weight
                
        return global_map
        
    def resample(self):
        '''
        Handle particle resampling here. In the initialization method, n_resamples
        is defined and represents the number of particles we want to resample
        because we aren't confident in their estimate of robot pose.
        
        This function resamples those particle positions according to where we
        think our robot is.

        Outputs:
            self.particles and self.weights is updated

        '''
        
        raise NotImplementedError
            
    @staticmethod
    def soft_max(x):
        temp = np.exp(x)
        return temp / temp.sum()