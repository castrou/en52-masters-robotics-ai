import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import spatialmath as sm

############## Robot Definition ################################################
H1=1
L1=0.6
L2=0.3
robot = rtb.Robot(rtb.ETS([
    rtb.ET.Rz(),
    rtb.ET.tz(H1),
    rtb.ET.tx(L1),
    rtb.ET.Rz(),
    rtb.ET.tx(L2),
    rtb.ET.Rx(180, unit='deg'),
    rtb.ET.tz()
]))

qlim = np.array([
    [-2*np.pi, 2*np.pi],
    [-2*np.pi, 2*np.pi],
    [0.3, 1]
], dtype=float).transpose()

robot.qlim = qlim

############## Function Definition #############################################
class HeartCurve:
    def __init__(self, g):
        self.g = g
        
    def __call__(self, num_pts, length):
        dt = (length/num_pts)
        series = np.empty((0,3))
        for t in np.arange(0, length + dt, dt):
            series = np.vstack((series, self.pt(t)))
        return series
            
    def pt(self, t):
        return np.array([self.x(t), self.y(t), self.z(t)])
        
    def x(self, t):
        return self.g*(2*np.cos(t) - np.cos(2*t))*0.01
    
    def y(self, t):
        return self.g*(2*np.sin(t) - np.sin(2*t))*0.01
    
    def z(self, t):
        return 0.1*self.g*t*0.01

############## MAIN ############################################################

if __name__ == '__main__':
    # Init
    num_pts= 50
    tf = 10
    dt = tf / num_pts
    path = HeartCurve(g=30) # scale g to SI units
        
    # Init robot
    robot.qr = [-2.54734843,  5.08853151,  0.93675007]
    home_pose = robot.fkine(robot.qr)
    joint_pose = robot.fkine_all(robot.qr)
    joint_pt = np.array([pose.t for pose in joint_pose[1:]])
    
    # Ideal Curve Plot
    curve = path(num_pts, tf)
    fig = plt.figure()
    fig.suptitle('Goal Curve')
    ax = fig.add_subplot(projection='3d')
    ax.scatter(curve[:,0], curve[:,1], curve[:,2], marker='o')
    ax.scatter(joint_pt[:,0], joint_pt[:,1], joint_pt[:,2], marker='o', color='blue')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    
    # Get joint angles
    q_vias = np.empty((0,3))
    traj_rel = []
    for i, pt in enumerate(curve[1:]):
        ik = robot.ik_NR(pt, tol=0.1, pinv=True)
        print(f'Point: {pt} Sol: {ik[0]}')
        q_vias = np.vstack((q_vias, ik[0]))
    
    # Fit curve
    q_traj = rtb.mstraj(q_vias, dt=dt/50, tacc=0.1, tsegment=[2*dt for _ in range(len(q_vias[:-1]))])
    
    # Plot
    ee_traj = np.empty((0,3))
    for q in q_vias:
        pt = robot.fkine(q)
        ee_traj = np.vstack((ee_traj, pt.t))
    
    fig = plt.figure()
    fig.suptitle('Robot Motion (end effector position)')
    ax = fig.add_subplot(projection='3d')
    ax.scatter(ee_traj[:,0], ee_traj[:,1], ee_traj[:,2], marker='o')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    plt.show()