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
    rtb.ET.tz(flip=True)
]))
qlim = np.array([
    [0, 2*np.pi],
    [0, 2*np.pi],
    [0.3, 1]
]).transpose()
robot.qlim = qlim

############## Function Definition #############################################
class HeartCurve:
    def __init__(self, g):
        self.g = g
        
    def __call__(self, num_pts, length):
        dt = (length/num_pts)
        series = np.empty((0,3))
        for t in np.arange(0, length, dt):
            series = np.vstack((series, self.pt(t)))
        return series
            
    def pt(self, t):
        return np.array([self.x(t), self.y(t), self.z(t)])
        
    def x(self, t):
        return self.g*(2*np.cos(t) - np.cos(2*t)) + 0.2
    
    def y(self, t):
        return self.g*(2*np.sin(t) - np.sin(2*t))
    
    def z(self, t):
        return 0.1*self.g*t

############## MAIN ############################################################

if __name__ == '__main__':
    # Init
    num_ik_via = 13 * 4
    tf = 10
    via_dt = tf / num_ik_via
    path = HeartCurve(g=0.3) # scale g to SI units
        
    # Init robot
    robot.qr = [0,  0,  1]
    home_pose = robot.fkine(robot.qr)
    joint_pose = robot.fkine_all(robot.qr)
    joint_pt = np.array([pose.t for pose in joint_pose[1:]])
    
    # Ideal Curve
    curve_resolution = 1000 # pts
    curve = path(curve_resolution, tf)
    curve_dindex = curve_resolution / num_ik_via

    # Get IK for segmented curve
    q_vias = np.empty((0,3))
    for i, pt in enumerate(curve[::int(curve_dindex)]):
        ik = robot.ikine_LM(sm.SE3.Trans(pt), mask=[1, 1, 1, 0, 0, 0], tol=0.001)
        q_vias = np.vstack((q_vias, ik.q))
        
    # Plot the plan
    ee_plan = np.empty((0,3))
    for q in q_vias:
        pt = robot.fkine(q)
        ee_plan = np.vstack((ee_plan, pt.t))
        
    fig = plt.figure()
    fig.suptitle('Pre-motion Plot')
    ax = fig.add_subplot(projection='3d')
    curve_plot = ax.plot3D(curve[:,0], curve[:,1], curve[:,2])
    ik_plot = ax.scatter(ee_plan[:,0], ee_plan[:,1], ee_plan[:,2], marker='o', color='blue')
    legend = ax.legend(['goal trajectory','proposed vias'], title="Legend")
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    
    # Fit curve
    # jtraj
    # segment_count = len(q_vias) - 1
    # q_traj = np.empty((1))
    # for i in range(segment_count):
    #     qd0 = [0.01*np.pi, 0.01*np.pi, 0.1]
    #     qdf = [0.01*np.pi, 0.01*np.pi, 0.1]
    #     if (i == 0):
    #         qd0 = [0, 0, 0]
    #     elif (i == segment_count - 1):
    #         qdf = [0, 0, 0]
    #     seg_traj =  rtb.jtraj(q_vias[i], q_vias[i+1], num_ik_via)
    #     q_traj = np.vstack((q_traj, seg_traj))
    # q_traj = [traj.q for traj in q_traj[1:,0]]
    
    # # Segmented mstraj
    # segment_count = 4
    # segment_length = int(num_ik_via / 4)
    # q_traj = np.empty(())
    # for i in range(segment_count):
    #     segment_vias = q_vias[:segment_length]
    #     segment_traj = rtb.mstraj(segment_vias, 0.1, tacc=0.01, qdmax=1)
    #     q_traj = np.vstack((q_traj, segment_traj))
    # q_traj = [traj.q for traj in q_traj[1:,0]]
    
    # Full mstraj
    q_traj = rtb.mstraj(q_vias, dt=0.1, tacc=0.1, qdmax=1)
    q_traj.plot()
    print(f'No. q points: {q_traj.q.shape[0]}')
    
    # Get end effector along q-trajectory
    ee_traj = np.empty((0,3))
    for q in q_traj.q:
        pt = robot.fkine(q)
        ee_traj = np.vstack((ee_traj, pt.t))
    
    # Plot the motion
    fig = plt.figure()
    fig.suptitle('Robot Motion (end effector position)')
    ax = fig.add_subplot(projection='3d')
    ax.plot3D(ee_traj[:,0], ee_traj[:,1], ee_traj[:,2], marker='o', markersize=1)
    ax.scatter(ee_plan[:,0], ee_plan[:,1], ee_plan[:,2], marker='o', color='blue')
    legend = ax.legend(['robot motion','planned vias'], title="Legend")
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
    plt.show()
    
    # Plot the trajectory
    fig = plt.figure()
    fig.suptitle('Robot trajcetory')