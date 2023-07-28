function [camera_frame] = body2camera(body_frame)

%Transfers coordinates in body frame to camera frame.
% The camera X axis = Body Y
% The camera Y axis = Body Z
% The camera Z axis = Body X

Tc = [0 1 0; 0 0 1; 1 0 0];
camera_frame = Tc*body_frame';


end

