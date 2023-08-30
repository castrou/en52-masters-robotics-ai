import math
import numpy as np
import roboticstoolbox as rtb

robot_ets = rtb.Robot(rtb.ETS([
    rtb.ET.Rz(jindex=0),
    rtb.ET.tz(0.1),
    rtb.ET.tx(0.06),
    rtb.ET.Rz(jindex=1),
    rtb.ET.tx(0.03),
    rtb.ET.Rx(180, unit='deg'),
    rtb.ET.tz(jindex=2)
]))

# define the initial joint angles
q = np.array([np.radians(15), np.radians(45), 0.2])

# define the timestep
dt = 0.1

# define the desired end-effector velocity - given in frame {0}
v = np.array([0.1, 0.2, 0.3, 0.2, -0.3, 0.1])

# loop over each time step
for t in np.arange(0,10,dt):
    # calculate the Jacobian at the current joint configuration
    J = robot_ets.jacob0(q)
    # calculate the pseudo inverse of the Jacobian - see np.linalg.pinv()
    J_inverse = np.linalg.pinv(J)
    # calculate the joint velocities for the desired end-effector velocity
    q_dot = J_inverse @ v
    # print time and joint velocities
    print("t =", t, ", q_dot =", q_dot)
    # update the joint angles
    q += dt*q_dot