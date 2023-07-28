function [ features ] = GetImageFeatures(imaging, references, noise, robotpose)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Finds image features given relative position between 
%               platform/camera and reference target including addition or
%               noise (if selected)

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Find Image Features (actual/true)
%**************************************************************************
features = ImageFeatures(imaging, robotpose, references.target);

%**************************************************************************
% Add Measurement Noise [s(t) = s(t) + v(t)]
%**************************************************************************
if noise.meas.on == 1          
    mnoise = normrnd(noise.meas.mu,noise.meas.std,size(features.position,1),size(features.position,2));
    features.position =  features.position + mnoise;
end



