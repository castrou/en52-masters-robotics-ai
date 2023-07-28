function [ noise ] = GetNoiseParameters(noiseon, imaging)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Sets noise/uncertainty parameters for simulation

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Define Noise/Uncertainty
%**************************************************************************
if strcmp(noiseon, 'ON')
    % Define Measurement/Sensor Noise Parameters
    noise.meas.on       = 1;         	    %   For adding noise to feature measurements 1 = ADDED NOISE || 0 = NO NOISE
    if strcmp(imaging.type, 'SPH') 
        noise.meas.std  = 0.5*pi/180; 	    %   Standard deviation of measurement noise (Allibert 2011)
        noise.meas.mu   = 0*pi/180;         %   Mean of measurement noise
    elseif strcmp(imaging.type, 'POL') 
        noise.meas.std  = [0.5*pi/180 5]; 	%   Standard deviation of measurement noise 
        noise.meas.mu   = [0*pi/180 0];     %   Mean of measurement noise
    else
        noise.meas.std  = 5;                %   Standard deviation of measurement noise
        noise.meas.mu   = 0;                %   Mean of measurement noise
    end
    
    % Define Process/Control Noise Parameters
  	noise.proc.on       = 0;         	    %   For adding process noise 1 = ADDED NOISE || 0 = NO NOISE
        noise.proc.std  = 0.02;             %   Standard deviation of process noise (Allibert 2011)
        noise.proc.mu   = 0*pi/180;         %   Mean of process noise
else
    noise.meas.on = 0;
    noise.proc.on = 0;
end



