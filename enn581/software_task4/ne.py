import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import sympy as sym

L1 = sym.Symbol('L1', real=True)
L1 = 0.4
# Init robot kinematics
robot = rtb.Robot(rtb.ETS([
    rtb.ET.Rx(90, unit='deg'),
    rtb.ET.Rz(),
    rtb.ET.tx(L1),
    rtb.ET.Ry(90, unit='deg'),
    rtb.ET.tz(),
    rtb.ET.Rz()
]))
print(robot)

def get_revolute_dynamics(joint_xfm: sm.SE3, joint_vel, joint_acc, prev_ang_vel, prev_ang_acc, prev_lin_vel, mass, moi):
    rot = joint_xfm.R
    trans = joint_xfm.t
    joint_vel_vec = joint_vel * np.array([[0], [0], [1]])
    joint_acc_vec = joint_acc * np.array([[0], [0], [1]])
    
    # Angular velocity of link
    angvel = rot @ prev_ang_vel + joint_vel_vec
    
    # Angular acceleration of link
    angacc = rot @ prev_ang_acc + np.cross((rot @ prev_ang_vel).transpose(), joint_vel_vec.transpose()).transpose() + joint_acc_vec
    
    # Linear acceleration of link at link frame
    linacc = rot @ (np.cross(prev_ang_acc.transpose(), trans).transpose() + np.cross(prev_ang_vel.transpose(), np.cross(prev_ang_vel.transpose(), trans)).transpose() + prev_lin_vel)
    
    # Linear acceleration of link at centroid frame
    # Point mass at distal end, do not adjust
    
    # Force acting on link at centroid  
    force = mass * linacc
    
    # Moment acting on link around centroid 
    moi = mass * np.linalg.norm(trans)**2 
    moment = moi * angacc + np.cross(angacc.transpose(), angacc.transpose()).transpose()

    return (angvel, angacc, linacc, force, moment)

def get_prismatic_dynamics(joint_xfm, joint_vel, joint_acc, prev_ang_vel, prev_ang_acc, prev_lin_acc, mass, moi):
    rot = joint_xfm.R
    trans = joint_xfm.t
    joint_vel_vec = joint_vel * np.array([[0], [0], [1]])
    joint_acc_vec = joint_acc * np.array([[0], [0], [1]])
    
    # Angular velocity of link
    angvel = rot @ prev_ang_vel
    
    # Angular acceleration of link
    angacc = rot @ prev_ang_acc
    
    # Linear acceleration of link at link frame
    linacc = rot @ (np.cross(prev_ang_acc.transpose(), trans) + np.cross(prev_ang_vel.transpose(), np.cross(prev_ang_vel, trans)).transpose() + prev_lin_acc) \
            + 2 * joint_vel_vec + joint_acc_vec
    
    # Linear acceleration of link at centroid frame
    # Point mass at distal end, do not adjust
    
    # Force acting on link at centroid
    force = mass * linacc
    
    # Moment acting on link around centroid 
    moment = moi * np.eye(3) * angacc + np.cross(angacc.transpose(), angacc.transpose())

    return (angvel, angacc, linacc, force, moment)

def ne_outwards(robot: rtb.Robot, mass, joint_pos, joint_vel, joint_acc, forces, moments):
    prev_ang_acc = np.array([[0], [0], [0]])
    prev_ang_vel = np.array([[0], [0], [0]])
    prev_lin_acc = np.array([[0], [0], [0]])
    
    for i, link in enumerate(robot.links):
        xfm = sm.SE3(np.transpose(link.A(joint_pos[i])))
        if link.isrevolute:
            (ang_vel, ang_acc, lin_acc, force, moment) = get_revolute_dynamics(xfm, joint_vel[i], joint_acc[i], prev_ang_vel, prev_ang_acc, prev_lin_acc, mass[i])
        else:
            (ang_vel, ang_acc, lin_acc, force, moment) = get_prismatic_dynamics(xfm, joint_vel[i], joint_acc[i], prev_ang_vel, prev_ang_acc, prev_lin_acc, mass[i])
        
        forces[i] = force
        moments[i] = moment
        
        prev_ang_acc = ang_acc
        prev_ang_vel = ang_vel
        prev_lin_acc = lin_acc
    return forces, moments

def ne_inwards(robot: rtb.Robot, joint_pos, forces, moments):
    force_balance = np.array([[0], [0], [0]])
    torque_balance = np.array([[0], [0], [0]])
    joint_torques = np.array([[0], [0], [0]])
    
    for i, link in list(enumerate(robot.links))[-2::-1]:
        xfm = link.A(joint_pos[i])
        f = forces[i] + xfm.R @ force_balance[i+1]
        n = moments[i] + xfm.R @ torque_balance[i+1] + np.cross(xfm.t, forces[i]) + np.cross(xfm.t, xfm.R @ force_balance[i+1])
        
        force_balance[i] = f
        torque_balance[i] = n
        if link.isrevolute:
            joint_torques[i] = np.transpose(n) * np.array([[0], [0], [1]])
        else:
            joint_torques[i] = np.transpose(f) * np.array([[0], [0], [1]])
            
    return joint_torques
        
if __name__ == '__main__':
    
    time = np.arange(0,1,0.01)
    m = [1, 1, 1]
    g = 9.81
    
    # traj
    q0 = [0, 0.2, 0]
    qf = [30, 0.4, 45]
    qtraj = rtb.jtraj(q0, qf, time)

    joint_torques = []
    for q, qd, qdd in zip(qtraj.q, qtraj.qd, qtraj.qdd):
        forces = np.array([[0,0,0], [0,0,0], [0,0,0]])
        moments = np.array([[0,0,0], [0,0,0], [0,0,0]])

        # outwards
        forces, moments = ne_outwards(robot, m, q, qd, qdd, forces, moments)
        
        # inwards
        joint_torques.append(ne_inwards(robot, q, forces, moments))
    print(joint_torques)