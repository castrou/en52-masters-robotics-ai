function [ robot, references] = Initialisation( imaging, timing, targettype, starttype )

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Initialised simulation including robot/agent position,
%               image feature locations and useful plotting parameters.
%               Allows selction of variable target and robot reference 
%               positions/poses. These include a set of fixed, nominal or
%               randomly generated positions/poses.

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Initial Target Position (Planar Object)
%**************************************************************************
target.type = targettype;
target.centre = [0 0 0];
switch target.type
    case 'square'
       target.height   = 0.5;
       target.width    = 0.5;
       % Target Position (NED) [TL TR BL BR]
       target.position = ...
          [target.centre + 0.5*[0 -target.width -target.height];...
           target.centre + 0.5*[0  target.width -target.height];
           target.centre + 0.5*[0 -target.width  target.height];
           target.centre + 0.5*[0  target.width  target.height]];
    case 'random'
        maxoffset  = 1; 
        minoffset  = -maxoffset;
        randoffset = (maxoffset - minoffset).*rand(8,1) + minoffset;
        target.position = ... 
          [target.centre + [0 randoffset(1:2,1)'];...
           target.centre + [0 randoffset(3:4,1)'];
           target.centre + [0 randoffset(5:6,1)'];
           target.centre + [0 randoffset(7:8,1)']];       
	case 'nonsym'
        maxoffset  = 1; 
        minoffset  = -maxoffset;
        target.position = ... 
           [target.centre + [0 -0.478006536132867 0.464951048313165];
            target.centre + [0 -0.674100069962791 0.842150444204442];
            target.centre + [0 -0.555553074669570 -0.832782565303993];
            target.centre + [0 -0.852596892515681 0.539103315324097]];  
    otherwise
        error('Incorrect target model selected! Select target.type = square');        
end
references.target = target;

%**************************************************************************
% Initial Robot/Camera Position & Orientation
%**************************************************************************
if strcmp(starttype,'nom')
    robotposition       = [-2 1 -3];                                                    % [x y z]                (m)
    robotattitude       = [0 0 10].*pi/180;                                             % [roll pitch yaw]       (rad)
    linewidth     = 2;
    linetype      = '-';
elseif strcmp(starttype, 'rnd')
    maxposoffset  = 3; 
    minposoffset  = -maxposoffset;
    robotposition = [randi([-3,-1],1,1) randi([-3,3],1,2)];
    %robotposition = (maxposoffset - minposoffset).*rand(1,3) + minposoffset;            % [x y z]                (m)                                     
    maxangoffset  = 45; 
    minangoffset  = -maxangoffset;
    robotattitude = ((maxangoffset - minangoffset).*rand(1,3) + minangoffset).*pi/180;  % [roll pitch yaw]       (rad)
  	linewidth     = 2;
    linetype      = '-';     
elseif strcmp(starttype, 'swt')
    robotposition       = [-1 1 2];
    robotattitude       = [0.176528833195147 -0.522780030400058 0.353586579381183]; % [10 -30 20]
    linewidth     = 2;
    linetype      = '-';
elseif strcmp(starttype, 'swt1')
    %Nominal, R=R(t), Stable, Switch 2 -> 4, V(k+1) < V(k)
    robotposition       = [-2,-1,3];
    robotattitude       = [0.0206568047678029,-0.745437644434203,0.0479188624359406]; % 1.2, -42.7, 2.7
    linewidth     = 1; %3
    linetype      = '-';
elseif strcmp(starttype, 'swt2')
    % Nominal, R=R(t), Stable,  No Switch 2 Correct, V(k+1) < V(k)
    robotposition       = [-1,2,-2];
    robotattitude       = [0.556884087324387,0.716275167760313,0.656826503961655]; % 31.9, 41.0, 37.6
    linewidth     = 1;
    linetype      = '-';
elseif strcmp(starttype, 'swt3')
    % Nominal, R=R(t), Stable,  No Switch 0 Correct, V(k+1) < V(k)
    robotposition       = [-1,-3,1];
    robotattitude       = [-0.746622972024719,-0.115727403544324,0.354914598010737]; %-42.8, -6.6, 20.4
    linewidth     = 3; 
    linetype      = '-'; %--
elseif strcmp(starttype, 'swt4')
    % Nominal, R=R(t), Stable,  No Switch 4 Correct, V(k+1) < V(k)
    robotposition       = [-2,1,-2];
    robotattitude       = [0.0922947730985236,0.105638215790722,0.0577193413554154]; % 5.3, 6.1, 3.3
    linewidth     = 1;
    linetype      = '-';
end
robot.plotting.linetype  = linetype;
robot.plotting.linewidth = linewidth;

robot.actual.pose     	= zeros(timing.N,6);
robot.actual.pose(1,:)	= [robotposition robotattitude];
robot.actual.range     	= zeros(timing.N,1);
robot.actual.range(1,1)	= norm(robot.actual.pose(1,1:3)-target.centre); 
robot.actual.control   	= zeros(timing.N,6);

% robot.nominal.pose    	= zeros(timing.N,6);
% robot.nominal.pose(1,:)	= robot.actual.pose(1,:);
% robot.nominal.range     = zeros(timing.N,1);
% robot.nominal.range(1,1)= robot.actual.range(1,1);
% robot.nominal.input     = zeros(timing.N,6);

robot.parameters.mass          = 1;                                                % Robot (arm) Mass(kg)
robot.parameters.mominertia    = [8.1e-3 8.1e-3 14.2e-3];                          % Robot Moment of Inertia (Jxx Jyy Jzz) 
%robot.mominertia    = [1 1 1]; 

%**************************************************************************
% Reference Camera Postion & Orientation
%**************************************************************************
referencesposition  = [-0.2 0 0];                                       % [x* y* z*]            (m)
referencesattitude  = [ 0 0 0 ].*pi/180;                            	% [roll* pitch* yaw*]   (rad)
references.pose     = [referencesposition referencesattitude];
references.features = ImageFeatures(imaging, references.pose, target);
references.range    = norm(referencesposition-target.centre);

%**************************************************************************
% Reference Controls
%**************************************************************************
% Velocity Controls (m/s, rad/s)
references.control.nominal 	= zeros(1,6);                           % Control Setpoint (radians)
references.control.max      = [ 0.2*ones(1,3)  5*ones(1,3)*pi/180];	% Vector for max u constraint (Allibert2010)
references.control.min      = [-0.2*ones(1,3) -5*ones(1,3)*pi/180];	% Vector for max u constraint (Allibert2010)

%% Plot Target
% figure(99)
% hold on
% for i=1:1:4
%     plot(target.position(i,2),target.position(i,3),'ko');
% end


