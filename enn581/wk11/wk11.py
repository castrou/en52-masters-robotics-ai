import numpy as np
import matplotlib.pyplot as plt
import spatialmath as sm

"""
draws an equilateral triangle at a given point (xC, yC) with a given orientation theta (in radians).
d is the distance of the vertex from the center of the triangle. 
"""

d = 1
b2 = np.deg2rad(-120)
b3 = np.deg2rad(120)
cx_w1 = 0
cy_w1 = d
cx_w2 = d * np.cos(b2)
cy_w2 = d * np.sin(b2)
cx_w3 = d * np.cos(b3)
cy_w3 = d * np.sin(b3)

m = 1
cI = m*d**2
M = np.array([[m, 0, 0],
    [0, m, 0],
    [0, 0, cI]])

def drawRobot(xC, yC, theta, d):
    A1 = np.matrix([[xC + 0],[yC + d/2],[1]])
    A2 = np.matrix([[xC + d*np.sqrt(3)/2],[yC - d/2],[1]])
    A3 = np.matrix([[xC - d*np.sqrt(3)/2],[yC - d/2],[1]])

    R = np.matrix([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])
    V1 = R*A1
    V2 = R*A2
    V3 = R*A3

    vertices = np.array([[V1[0,0], V1[1,0]], [V2[0,0], V2[1,0]], [V3[0,0], V3[1,0]]])
    return vertices

def jacob(th):
    return sm.SO3.Rz(th)

def get_forcetorque(F1, F2, F3):
    global d, b2, b3, cx_w1, cy_w1, cx_w2, cy_w2,  cx_w3, cy_w3
    cf_x = F1 + F2 * np.cos(b2) + F3 * np.cos(b3)
    cf_y = F2 * np.sin(b2) + F3 * np.sin(b3)
    cn_z =  F1 * -cy_w1 + \
            F2 * (cy_w2 * np.cos(b2) + cx_w2 * np.sin(b2)) + \
            F3 * (-cy_w3 * np.cos(b3) - cx_w3 * np.sin(b3))
            
    return cf_x, cf_y, cn_z

def get_dzeta(u, v, dth, cf_x, cf_y, cn_z):
    global m, cI
    # m*ddu + -m*dth*v = cf_x
    ddu = (cf_x / m) + dth*v
    # m*ddv + m*dth*u = cf_y
    ddv = (cf_y / m) + dth*u
    # cI*ddth = cn_z
    ddth = cn_z / cI
    return ddu, ddv, ddth
    
    

x = 0
y = 0
th = 0
dx = 0
dy = 0
dth = 0
u = 0
v = 0
dt = 0.01
time = np.arange(0, 5, dt)
F = [1, 0.5, 1]

for t in time:
    J = jacob(th)
    cf_x, cf_y, cn_z = get_forcetorque(F[0], F[1], F[2])
    ddu, ddv, ddth = get_dzeta(u, v, dth, cf_x, cf_y, cn_z)
    ddeta = J * np.transpose(np.array([ddu, ddv, ddth]))

    ddx = ddeta[0][0]
    ddy = ddeta[1][0]
    ddth = ddeta[2][0]
    
    dx += ddx * dt
    dy += ddy * dt
    dth += ddth * dt
    
    x += dx * dt 
    y += dy * dt
    th += dth * dt

    X = drawRobot(x, y, th, d)

    plt.figure()
    plt.scatter(X[:, 0], X[:, 1], s = 170)

    t1 = plt.Polygon(X[:3,:])
    plt.gca().add_patch(t1)

plt.show()

