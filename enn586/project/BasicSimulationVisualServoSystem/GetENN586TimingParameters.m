function [ timing ] = GetENN586TimingParameters(Total_time)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Sets timing parameters for simulation

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Simulation Parameters & Timing
%**************************************************************************
timing.T        = Total_time;           % Total time (s)
timing.dt       = 0.1;         % Sampling Interval (10-50Hz)
timing.freq     = 1/timing.dt;
timing.N        = timing.T * timing.freq;

timing.samples  = (1:1:timing.N);
timing.t        = timing.samples./timing.freq;

timing.switch.samples = round([1 2 3].*timing.N/4);

end

