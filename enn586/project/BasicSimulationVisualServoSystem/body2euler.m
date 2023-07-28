function [euler_mtx] = body2euler(PHI, THETA)

% Generate transformation matrix to convert angular rates in body 
% frame to inertial/fixed frame (NED) based on Euler angles. 

euler_mtx1 = [1 sin(PHI)*tan(THETA) cos(PHI)*tan(THETA)];
euler_mtx2 = [0 cos(PHI) -sin(PHI)];
euler_mtx3 = [0 sin(PHI)*sec(THETA) cos(PHI)*sec(THETA)];

euler_mtx = [euler_mtx1;euler_mtx2;euler_mtx3];


end

