% ENN586 Main loop
% ENN586 Student you should not change this code. 

%% Main Loop
%close all
for i=1:1:Timing.N

    % Get (Associated and non-degraded) Image Features (correct number and order w.r.t target features)
    Robot.actual.features(i)  	= GetImageFeatures(Imaging, References, Noise, Robot.actual.pose(i,:));

    % Get Visual Control
    [YourVariables,Robot.control(i,:), Robot.error(i)] = GetENN586ImageControl(YourVariables,Imaging, References, Noise, Control, ENN586error, Robot.actual.features(i));

    % Apply Visual Control - State Update (angular rates)
    Robot.actual.transform.Ra(:,:,i) = body2euler(Robot.actual.pose(i,4),Robot.actual.pose(i,5));
    Robot.actual.velocity(i,4:6)     = Robot.actual.transform.Ra(:,:,i)*Robot.control(i).actual(4:6)';
    Robot.actual.pose(i+1,4:6)       = Robot.actual.pose(i,4:6) + Robot.actual.velocity(i,4:6).*Timing.dt;

    % Apply Visual Control - State Update (position rates)
    Robot.actual.transform.Rp(:,:,i) = body2ned(Robot.actual.pose(i,4), Robot.actual.pose(i,5), Robot.actual.pose(i,6));

    if strcmp(ENN586error.wind.status, 'ON')
        Robot.actual.velocity(i,1:3)     = Robot.actual.transform.Rp(:,:,i)*Robot.control(i).actual(1:3)'+ENN586error.wind.effect;
    else
        Robot.actual.velocity(i,1:3)     = Robot.actual.transform.Rp(:,:,i)*Robot.control(i).actual(1:3)';
    end

    Robot.actual.pose(i+1,1:3)       = Robot.actual.pose(i,1:3) + Robot.actual.velocity(i,1:3).*Timing.dt;
    Robot.actual.range(i+1)          = norm(Robot.actual.pose(i+1,1:3)-References.target.centre);
    %--------------------------------------------------------------------------------------
end