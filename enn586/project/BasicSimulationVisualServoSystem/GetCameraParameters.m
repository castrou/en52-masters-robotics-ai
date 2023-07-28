function [ imaging ] = GetCameraParameters( imagingtype )

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Sets camera parameters for visual servoing task. This
%               includes the number of images features (1-4), some 
%               intrinsic parameters and advanced features (feature
%               shuffling and degradation). Advanced features only active
%               with auxillary code (contact aaron.mcfadyen@qut.edu.au)


% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Define Number Image Features (that robot would sense)
%**************************************************************************
imaging.numfeatures = 4;
imaging.featmin     = 1;
imaging.featmax     = 4;

%**************************************************************************
% Shuffle Image Feature Order (like robot would sense)
%**************************************************************************
%imaging.random = 'ON';
imaging.random = 'OFF';

%**************************************************************************
% Degrade Number Features (like occlusions from robot)
%**************************************************************************
%imaging.featdegrade = 'ON';
imaging.featdegrade = 'OFF';

%**************************************************************************
% Define Camera Paramters (intrinsic/extrinsic)
%**************************************************************************
imaging.type = imagingtype;
switch imaging.type
    
    case 'POL'
        imaging.f    = 10/1000;
    
    case 'SPH'
        imaging.f    = 10/1000;
    
    case 'CRT'
        imaging.f    = 10/1000;
end

