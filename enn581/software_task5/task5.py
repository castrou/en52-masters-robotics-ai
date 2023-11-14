import numpy as np
import spatialmath as sm

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def drawRobot(xC, yC, theta, a, b):
    A1 = np.matrix([[xC - 2*b],[yC + a],[1]])
    A2 = np.matrix([[xC + 2*b],[yC + a],[1]])
    A3 = np.matrix([[xC - 2*b],[yC - a],[1]])
    A4 = np.matrix([[xC + 2*b],[yC - a],[1]])

    R = np.matrix([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])
    V1 = R*A1
    V2 = R*A2
    V3 = R*A3
    V4 = R*A4

    vertices = np.array([[V1[0,0], V1[1,0]], 
                         [V2[0,0], V2[1,0]], 
                         [V4[0,0], V4[1,0]],
                         [V3[0,0], V3[1,0]],
                         [V1[0,0], V1[1,0]]])
    return vertices

NUM_WHEELS = 4

X = 0
Y = 1
THETA = 2

class WheelConfig:
    def __init__(self, r, gamma, beta, cx, cy):
        self.gamma = gamma
        self.beta = beta
        self.r = r
        self.cxw = cx
        self.cyw = cy
        
    def get_W(self, th):
        # wheel geometry
        W_geom_term = np.array([1/self.r, np.tan(self.gamma)/self.r])
        # wheel orientation (wrt chassis)
        W_rot_term = np.array([[np.cos(self.beta), np.sin(self.beta)], 
                                [-np.sin(self.beta), np.cos(self.beta)]])
        # wheel position (wrt chassis)
        W_pos_term = np.array([[1, 0, -self.cyw], 
                                [0, 1, self.cxw]])
        # robot orientation (wrt world)
        cos_th = np.cos(th)[0]
        sin_th = np.sin(th)[0]
        R_rot_term = np.array([[cos_th, sin_th, 0.0],
                               [-sin_th, cos_th, 0.0],
                               [0.0, 0.0, 1.0]])
        return W_geom_term @ W_rot_term @ W_pos_term @ R_rot_term
    
class RobotConfig:  
    def __init__(self, init_q, init_dq, init_ddq, a, b, r, init_phi=np.array([[0],[0],[0],[0]]), init_F=np.array([[0],[0],[0],[0]]), m=1, I=1, Q=1):
        self.q = init_q
        self.dq = init_dq
        self.ddq = init_ddq
        self.a = a
        self.b = b
        self.r = r
        self.gamma = [np.deg2rad(45), np.deg2rad(-45), np.deg2rad(-45), np.deg2rad(45)]
        self.beta = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]
        self.wheels = [WheelConfig(r, self.gamma[0], self.beta[0], cx=-b, cy=a),     # Wheel 1
                  WheelConfig(r, self.gamma[1], self.beta[1], cx=b, cy=a),      # Wheel 2
                  WheelConfig(r, self.gamma[2], self.beta[2], cx=-b, cy=-a),    # Wheel 3
                  WheelConfig(r, self.gamma[3], self.beta[3], cx=b, cy=-a)]     # Wheel 4
        
        # kinematics
        self.zeta = np.array([[0], [0], [0]], dtype='float64')
        self.phi = init_phi

        # dynamics
        self.F = init_F
        self.m = m
        self.I = I
        self.M = np.array([[m, 0, 0], 
                           [0, m, 0], 
                           [0, 0, I]])
        
        # update function
        self.update = self.q1_update if Q==1 else self.q2_update
    
    def jacob(self, th):
        return sm.SE2.Rot(th).A
    
    def q1_update(self, dt):
        theta = self.q[THETA]
        
        # Wheel (angular) velocity to chassis velocity
        W = np.array([wheel.get_W(theta) for wheel in self.wheels]).transpose() / NUM_WHEELS
        self.zeta = W @ self.phi
        # Chassis velocity to global inertial frame
        self.dq = self.jacob(theta) @ self.zeta 
        
        # update
        self.q += self.dq * dt
        self.dq += self.ddq * dt
        
        return self.q
                        
    def q2_update(self, dt):
        # shorten function and variable calls
        b = self.b
        a = self.a
        cos = np.cos
        sin = np.sin
        wheel1, wheel2, wheel3, wheel4 = self.wheels
        gamma1, gamma2, gamma3, gamma4, = wheel1.gamma, wheel2.gamma, wheel3.gamma, wheel4.gamma
        
        # get chassis force-torque
        wc_force_rel = np.array([[sin(gamma1), sin(gamma2), sin(gamma2), sin(gamma2)],
                                 [cos(gamma1), cos(gamma2), cos(gamma2), cos(gamma2)],
                                 [b*cos(gamma1)-a*sin(gamma1), b*cos(gamma2)-a*sin(gamma2), -b*cos(gamma3)+a*sin(gamma3), -b*cos(gamma4)+a*sin(gamma4)]])
        cfx, cfy, cnz = wc_force_rel @ self.F
        cfx, cfy, cnz = cfx[0], cfy[0], cnz[0]
        
        # get chassis accel
        u, v, dth = self.zeta.transpose()[0]
        self.dzeta = np.array([[(cfx / self.m)  + dth * v],
                               [(cfy/self.m) + dth * u],
                               [cnz / self.I]])
        # update zeta
        self.zeta += self.dzeta * dt
        
        # Chassis velocity to global inertial frame
        self.dq = self.jacob(self.q[THETA]) @ self.zeta 
        
        # update
        self.q += self.dq * dt
        
        return self.q
        
def q1_bot(init_q, init_dq, init_ddq):
    a = 0.5
    b = 0.2
    r = 0.01
    phi = np.array([[np.deg2rad(10)], [np.deg2rad(20)], [np.deg2rad(30)], [np.deg2rad(40)]])
    
    return RobotConfig(init_q, init_dq, init_ddq, a, b, r, init_phi=phi)
    
def q2_bot(init_q, init_dq, init_ddq):
    a = 0.5
    b = 0.2
    r = 0.01
    m = 6
    I = 0.25
    
    F = np.array([[10],[20],[20],[10]])

    return RobotConfig(init_q, init_dq, init_ddq, a, b, r, init_F=F, Q=2)


def simulate_robot(q=1):
    # initial robot pose
    eta = np.array([[0],[0],[0]], dtype='float64') # x, y, theta
    eta_history = eta

    # define robot parameters
    deta = np.array([[0],[0],[0]], dtype='float64')
    ddeta = np.array([[0],[0],[0]], dtype='float64')
    robot = q1_bot(init_q=eta, init_dq=deta, init_ddq=ddeta) if q == 1 else \
        q2_bot(init_q=eta, init_dq=deta, init_ddq=ddeta)

    # define time step
    dt = 0.01

    # loop from t=0 to t=10 with increments of dt
    time = 0
    while time < 10:
        # simulate motion to update eta
        eta = robot.update(dt)
        # update eta_history
        eta_history = np.concatenate((eta_history, eta), axis=1)
        # update time
        time = time + dt

    # return robot pose history
    return eta_history


def plot_traj(robot_trajectory):    
    # Create a figure and axis for the animation
    fig, ax = plt.subplots(1,2, figsize=(12,6))
    lim_val = 4
    ax[0].set_xlim(-lim_val, lim_val)
    ax[0].set_ylim(-lim_val, lim_val)
    ax[1].set_xlim(-lim_val, lim_val)
    ax[1].set_ylim(-lim_val, lim_val)
    
    ax[0].set_title('Path')
    ax[1].set_title('Robot Pose (animation is bugged)')

    # Create a line object for the robot's path
    traj, = ax[0].plot([], [], 'r-', lw=2)
    bot, = ax[1].plot([], [], 'b-', lw=2)
    
    # Function to initialize the plot
    def init():
        bot.set_data([], [])
        traj.set_data([], [])
        return bot,

    # Function to update the plot for each frame of the animation
    def animate(i):
        x, y, theta = robot_trajectory[:,i]
        vertices = drawRobot(x, y, theta, a=0.5, b=0.2)
        plt_x, plt_y = vertices[:, 0], vertices[:, 1]
        bot.set_data(plt_x, plt_y)
        traj.set_data(robot_trajectory[0,:i+1], robot_trajectory[1,:i+1])
        return ax,

    # Create the animation
    ani = FuncAnimation(fig, animate, interval=10, frames=robot_trajectory.shape[1], init_func=init, blit=True)

    plt.show()

if __name__ == '__main__':
    
    # set question index to change which sim is used
    robot_trajectory = simulate_robot(2)
    plot_traj(robot_trajectory.transpose())
