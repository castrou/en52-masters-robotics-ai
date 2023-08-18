import roboticstoolbox as rtb
import matplotlib.pyplot as plt
from spatialmath.base import *

# import the puma robot arm model
puma = rtb.models.DH.Puma560()
# plot the initial configuration of the robot
figure1 = plt.figure()
puma.plot(puma.qz, 'pyplot', fig=figure1)
# set a desired target configuration
T = transl(0.5, 0.1, 0.0) @ rpy2tr(0, 180, 0, 'deg')
print(T)

# solve the inverse kinematics for the desired target
# <== change the parameter below to see other variations ==>
sol = puma.ikine_NR(T, 'f') # we are forcing the robot to go into a right handed configuration
puma.plot(sol.q, 'pyplot', fig=figure1)
trplot(T) # plot the desired target to confirm
plt.draw() # draw the plot
plt.pause(20) # show it for 5 seconds