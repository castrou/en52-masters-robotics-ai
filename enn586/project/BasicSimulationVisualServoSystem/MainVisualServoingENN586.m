%==========================================================================
%                    BASIC VISUAL SERVOING SIMULATOR 
%==========================================================================

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Basic visual servoing simulator using classical visual
%               control approaches to move a generic platform. Users can 
%               select the controllable deg. of freedom (velocity), camera 
%               model and addition of uncertaintly/noise for a range of 
%               intial conditions. Euler approximation is used to update
%               robot position without specifc platform dynamics.


% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au
% Modifed May 2023 for teaching use in ENN586, contact: j2.ford@qut.edu.au

%% Clear Workspace, Command Line and Plots
clear all
close all
clc

rng(42);

%%
% There are a number of test cases for you to examine
% test_case=1; %No error, no measurement noise case
% test_case=2; %Measure error is ON. No other error sources.
% test_case=3; %Range error is ON. No other error sources.
% test_case=4; %Feature error is ON. No other error sources.
% test_case=5; %Wind error is ON. No other error sources.
% test_case=6; %Measurement, range, feature, Wind errors are all ON.  

% There are 4 start pose cases for you to examine
% start_case=1; % etc.

%  ENN568 Student: You may change the next 3 lines to change test case, start pose, and simluation length 
test_case=1;    % there are 6 cases
start_case=1;   % there are 4 start pose case

simulation_length=100;  %length in s.
YourVariables.T = simulation_length;
YourVariables.dt = 0.1;

% pid controller
simulation_length=100;  %length in s.
YourVariables.var1=zeros(6,1);   % a structure (or variables) you can pass between functions.  You can relabel the var1 field and add fields as you like.
YourVariables.error_old=zeros(1,1);
YourVariables.first=1;
YourVariables.gainI=-0.0009;
YourVariables.gainD=-0.3;
YourVariables.gainP=0.3;
YourVariables.varv=YourVariables.var1;

% Kalman Filter
YourVariables.kf.A = eye(8);
YourVariables.kf.R = 25;
YourVariables.kf.Q = diag(0.01*ones([8 1]));
YourVariables.kf.H = eye(8);
YourVariables.kf.P_prev = 1000 * eye(8);
YourVariables.kf.B_prev = 0.01*rand([8 6]);
YourVariables.kf.u_prev = zeros([6 1]);
YourVariables.kf.predict = @kalman_predict;
YourVariables.kf.measure = @kalman_measure;

x_0 = 5*(rand([8 1]) - rand([8 1]));
YourVariables.kf.e_est = x_0;

kf = YourVariables.kf;
[kf.e_pred kf.P_pred] = kf.predict(kf.A, kf.e_est, kf.B_prev, kf.u_prev, kf.P_prev, kf.Q);
YourVariables.kf = kf;

%  ENN568 Student: Do not modify anything below, except you may edit inside ENN586YourControl.m

ENN586Intialisations; % a script function that build environment from above information.

%% Main Loop
ENN586MainLoop % a script file executing the dynamics (calls ENN586YourControl that students compose).

%% Plotting and Results.
close all
DisplayENN586Results(Timing, Imaging, References, Noise, Control, Robot, ENN586error, 'OFF');

%% Function Definition
%% Kalman Time Update
function [x_pred, P_pred] = kalman_predict(A_prev, x_prev, B_prev, u_prev, P_prev, Q)
    x_pred = (A_prev*x_prev) + (B_prev * u_prev);
    P_pred = A_prev*P_prev*A_prev' + Q;
end

%% Kalman Meas Update
function [x_est, P, K] = kalman_measure(x_pred, P_pred, z, H, R)
    K = P_pred*H'*inv(H*P_pred*H'+R);
    x_est = x_pred + (K*(z-H*x_pred));
    P = (eye(8) - K*H)*P_pred;
end