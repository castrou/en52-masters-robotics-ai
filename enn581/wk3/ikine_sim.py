import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from spatialmath.base import *
import math

robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.2433, alpha=-math.pi/2),
    rtb.RevoluteDH(offset=-math.pi/2, alpha=math.pi, a=0.200),
    rtb.RevoluteDH(offset=-math.pi/2, alpha=math.pi/2, a=0.087),
    rtb.RevoluteDH(d=0.2276, alpha=math.pi/2),
    rtb.RevoluteDH(alpha=-math.pi/2),
    rtb.RevoluteDH(d=0.0615)], name="enn581")

# set a desired target configuration
figure1 = plt.figure()
T = transl(0.5, 0.1, 0.0) @ rpy2tr(0, 180, 0, 'deg')
print(T)

# solve the inverse kinematics for the desired target
# <== change the parameter below to see other variations ==>
sol = robot.ikine_NR(T, 'f') # we are forcing the robot to go into a right handed configuration
robot.plot(sol.q, 'pyplot', fig=figure1)
trplot(T) # plot the desired target to confirm
plt.draw() # draw the plot
plt.pause(20) # show it for 5 seconds