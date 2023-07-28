function [ control ] = GetControlParameters(type, platform, imaging)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Sets control parameters including controllable DoF
%               and gain. Currently supports only 'TYPE=CLASSICAL' visual 
%               servoing but multiple DoF including those consistent with a
%               a 'PLATFORM=QUADROTOR' platform. Specifc platform dynamics
%               not included (can be added to MAIN function)
%               

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Set Control Type and Platform
%**************************************************************************
control.type        = type;
control.platform    = platform;

%**************************************************************************
% Determine Controllable Degrees of Freedom
%**************************************************************************
switch imaging.numfeatures(end)
    case 1
        control.dof = [1 2];
    case 2
        control.dof = [1 2 3 6];
	case {3,4}
        if strcmp(control.platform, 'QUAD')
            control.dof = [1 2 3 6];
        else
            control.dof = [1 2 3 4 5 6];
        end
    otherwise            
        error(horzcat('Cannot control with ',num2str(imaging.numfeatures),' features'))
end

%**************************************************************************
% Set Controller Parameters
%**************************************************************************
if strcmp(control.type, 'CLASSICAL') 
    control.gain    = 0.15;
else
    error('NON-CLASSICAL control approaches not supported. Contact aaron.mcfadyen@qut.edu.au')
end

