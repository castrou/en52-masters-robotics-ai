function [body_frame] = camera2body(camera_frame)

%Transfers coordinates in camera frame to body frame.
% The Body Y axis = camera X
% The Body Z axis = camera Y 
% The Body X axis = camera Z

Tc = [0 0 1; 1 0 0; 0 1 0];
body_frame = Tc*camera_frame';


end

