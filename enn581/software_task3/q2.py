import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
import spatialmath as sm

############## Robot Definition ################################################
robot = rtb.Robot(rtb.ETS([
    rtb.ET.Rz(jindex=0),
    rtb.ET.tz(1),
    rtb.ET.tx(0.6),
    rtb.ET.Rz(jindex=1),
    rtb.ET.tx(0.3),
    rtb.ET.Rx(180, unit='deg'),
    rtb.ET.tz(jindex=2)
]))

qlim = np.array([
    [0, 2*np.pi],
    [0, 2*np.pi],
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
        return self.g*(2*np.cos(t) - np.cos(2*t))
    
    def y(self, t):
        return self.g*(2*np.sin(t) - np.sin(2*t))
    
    def z(self, t):
        return 0.1*self.g*t

############## MAIN ############################################################

if __name__ == '__main__':
    # Init
    num_pts= 50
    tf = 10
    dt = tf / num_pts
    path = HeartCurve(g=0.3) # scale g to SI units
        
    # Ideal Curve Plot
    curve = path(num_pts, tf)
    fig = plt.figure()
    fig.suptitle('Goal Curve')
    ax = fig.add_subplot(projection='3d')
    ax.scatter(curve[:,0], curve[:,1], curve[:,2], marker='o')
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    
    # Initialise robot pose
    robot.qr = [0, 0, 0.3]
    home_pose = robot.fkine(robot.qr)
    xrange = max(curve[:,0]) - min(curve[:,0])
    yrange = max(curve[:,1]) - min(curve[:,1])
    base_pose = home_pose @ sm.SE3.Trans([xrange/2, yrange/2, 0])
    print(base_pose)
    
    # Get joint angles
    q_vias = np.empty((0,3))
    for pt in curve:
        ik = robot.ik_NR(np.transpose(base_pose) * sm.SE3.Trans(pt))
        print(ik[0])
        q_vias = np.vstack((q_vias, ik[0]))
    
    # Fit curve
    q_traj = rtb.mstraj(q_vias, dt=dt, tacc=1, tsegment=[dt for _ in range(len(q_vias)-1)])
    
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