function [  ] = DisplayENN586Results(timing, imaging, references, noise, control, robot,  ENN586error, animations)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Displays results for visual servoing task. This includes
%               plots (robot position, feature poistion, features error, 
%               control etc.,) and animations (if selected).

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%% Dsiplay Servoing Details

disp('******************* ENN586 Setting Details ***************************')
disp(horzcat('Simulation legnth: ',num2str(timing.T),' s') )

if strcmp(ENN586error.noise.status, 'ON')
%    disp(horzcat('Measurement noise mean: ',num2str(ENN586error.noise_mu),'. Measurement noise std: ',num2str(ENN586error.noise_std)) )
 disp(horzcat('Measurement noise is ON. Measurement noise mean: ',num2str(ENN586error.noise_para.meas.mu),'. Measurement noise std: ',num2str(ENN586error.noise_para.meas.std)) )
else
    disp('Measurement noise is OFF')
end

if strcmp( ENN586error.range.status, 'ON')
    disp(horzcat('Range error is ON. Range Error Type: ', ENN586error.range.type,'. Range Error Size: ',num2str(ENN586error.range.size)))
else
    disp('Range error is OFF')
end
% if strcmp(range_error.status, 'ERROR')
%     disp(horzcat('Range Error Type: ',Error.range_type,', Invalid type set, ran with no range error. '))
% end

if strcmp(ENN586error.feature.status, 'ON')
    disp(horzcat('Feature error is ON. Feature Error Size: [',num2str(ENN586error.feature.offset),'];') )
else
    disp('Feature error is OFF')
end

if strcmp(ENN586error.wind.status, 'ON')
    disp(horzcat('Wind is ON. Wind Size:  [',num2str(ENN586error.wind.effect'),']; m/s') )
else
    disp('Wind is OFF')
end

if strcmp(ENN586error.control.status , 'ON')
    disp(horzcat('Control disturbance is ON. Size: ',num2str(ENN586error.control.disturbance'),'; m/s') )
else
    disp('Control disturbance is OFF')
end

 disp(horzcat('Robot start pose is:  [',num2str(ENN586error.startpose),']; (m,m,m,rad,rad,rad)') )
%  disp(horzcat('Reference target is:  [',num2str(ENN586error.references.features.position),']; (m,m,m,rad,rad,rad)') )
disp('                                                               ')

% disp('******************* Servoing Details (less important in ENN586) ***************************')
% disp(horzcat('Nominal Control Approach: ',control.type));
% disp(horzcat('Camera Features: ',imaging.type));
% if strcmp(imaging.random, 'ON')
%     disp(horzcat('Scrambled Features: ',imaging.random,' (Image features recorded in random order --> similar to how a robot/camera would work before processing image features)'))
% else
%     disp(horzcat('Scrambled Features: ',imaging.random,' (Image features recorded in correct order --> similar to how a robot/camera would work, after processing image features)'))
% end
% disp(horzcat('Measurement Noise: ',num2str(noise.meas.on)));
% disp(horzcat('Process Noise: ',num2str(noise.proc.on)));
% 
% % if strcmp( ENN586error.range_status, 'ON')
% %     disp(horzcat('Range Error Type: ', ENN586error.range_type,', Range Error Size: ',num2str(ENN586error.range_size)))
% % else
% %     disp('Range error is OFF')
% % end
% % % if strcmp(range_error.status, 'ERROR')
% % %     disp(horzcat('Range Error Type: ',Error.range_type,', Invalid type set, ran with no range error. '))
% % % end
% % 
% % 
% % if strcmp(ENN586error.feature_status, 'ON')
% %     disp(horzcat('Feature Error Size: [',num2str(ENN586error.feature_offset),'];') )
% % else
% %     disp('Feature error is OFF')
% % end
% 
if strcmp(imaging.type,'SPH')
    xlab = 'Azimuth (deg)';
    ylab = 'Colatitude (deg)';
elseif strcmp(imaging.type,'CRT')
    xlab('u (pix)');
    ylab('v (pix)');
elseif strcmp(imaging.type,'POL')
    xlab('Angle A (deg)');
    ylab('Radius (pix/m)');
else
    xlab('Feature A (deg/pix/m)');
    ylab('Feature B (deg/pix/m)');
end
%  
%     
% disp('                                                               ')

%% Display Performance Data
disp('******************* Some Performance Details ***************************')
actualtrans         = robot.actual.velocity(:,1:3).^2;
actualrotat        	= robot.actual.velocity(:,4:6).^2;

% Calculate Actual Control Effort/Energy ()
energy.actualtrans       = sum(0.5*robot.parameters.mass*actualtrans,1);
energy.actualrotat(1,1)  = sum(0.5*robot.parameters.mominertia(1)*actualrotat(:,1));
energy.actualrotat(1,2)  = sum(0.5*robot.parameters.mominertia(2)*actualrotat(:,2));
energy.actualrotat(1,3)  = sum(0.5*robot.parameters.mominertia(3)*actualrotat(:,3));
energy.actualtotal = sum([energy.actualtrans energy.actualrotat]);

% Display Control Effort/Energy
disp(horzcat('Total Control Energy/Effort (actual): ',num2str(energy.actualtotal),' Joules'));                                                        
disp(horzcat('Total Control Energy/Effort (actual-trans): ',num2str(energy.actualtrans),' Joules'));                                                      
disp(horzcat('Total Control Energy/Effort (actual-rot): ',num2str(energy.actualrotat),' Joules'));
disp('                                                               ')

%% Arrange Colors
linetrans = 1;
linetype  = robot.plotting.linetype;
linewidth = robot.plotting.linewidth;


%% Display Performance Data
disp('******************* Final Location Details ***************************')

%% Plot Servoing Results
%**************************************************************************
% Robot Position
%**************************************************************************
figure(1)           
hold on
% Plot Position
plot3(robot.actual.pose(:,1),robot.actual.pose(:,2),robot.actual.pose(:,3),linetype,'Color',[0 0 0 linetrans],'LineWidth',linewidth)
plot3(robot.actual.pose(1,1),robot.actual.pose(1,2),robot.actual.pose(1,3),'k.','MarkerSize',20)
plot3(references.pose(1,1),references.pose(1,2),references.pose(1,3),'k*','MarkerSize',6)
h =  findobj(gcf,'Type','axes','Tag','legend');

% Plot Target
for i=1:1:size(references.target.position,1) 
    h=plot3(references.target.position(i,1),references.target.position(i,2),references.target.position(i,3),'ko','MarkerSize',10); h.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
title('Target and Robot Position','interpreter','latex','FontSize',14)
xlabel('x (m)','interpreter','latex','FontSize',14);
ylabel('y (m)','interpreter','latex','FontSize',14);
zlabel('z (m)','interpreter','latex','FontSize',14)
%text(robot.pose(1,1)+1, robot.pose(1,2)-1,horzcat('Energy: ',num2str(energy.total)))
%hl1 = legend('$\hat{\sigma}$', '$\hat{\gamma}$','$\sigma$','$\gamma$');
%set(hl1,'Interpreter','latex');
hold off

disp(horzcat('Final robot position error: ',num2str(rms([references.pose(1,3)-robot.actual.pose(end,3), references.pose(1,2)- robot.actual.pose(end,2),references.pose(1,1)- robot.actual.pose(end,1)])),' (m)'));  

%**************************************************************************
% Image Features
%**************************************************************************
figure(2)           
hold on
c=hsv(size(references.features.position,1));
value = reshape([robot.actual.features(:).position],4,2,timing.N);
if strcmp(imaging.type, 'SPH')
	value = value*180/pi;
    referencesposition = references.features.position*180/pi;
end
for i=1:1:size(references.features.position,1)    
    plot(referencesposition(i,2),referencesposition(i,1),'o','Color',c(i,:),'MarkerSize',10);
    plot(reshape(value(i,2,:),[],1) ,reshape(value(i,1,:),[],1),linetype,'Color',[c(i,:) linetrans],'LineWidth',linewidth);
    plot(value(i,2,1) ,value(i,1,1),'.','Color',c(i,:),'MarkerSize',14);
    plot(value(i,2,end) ,value(i,1,end),'*','Color',c(i,:),'MarkerSize',6);
    disp(horzcat('Feature ',num2str(i),' Final image position error: ',num2str(rms([referencesposition(i,2)- value(i,2,end),referencesposition(i,1)- value(i,1,end)])),' (deg)'));  
end
%hl1 = legend('$\hat{\sigma}$', '$\hat{\gamma}$','$\sigma$','$\gamma$');
%set(hl1,'Interpreter','latex');
xlabel(xlab,'interpreter','latex','FontSize',14);
ylabel(ylab,'interpreter','latex','FontSize',14);
title('Image Features','interpreter','latex','FontSize',14)
ax = gca;
ft2lim = ax.YLim;
ft1lim = ax.XLim;
hold off

%**************************************************************************
% Control / Input
%**************************************************************************
figure(3)           
hold on
value = [];
value = reshape([robot.control(:).actual],6,[]);
%c     = hsv(size(value,1));
c = [1 0 0;0 1 0;0 0 1;1 0 1;0 0 0;0 1 1];
for i=1:1:size(value,1)    
    plot(timing.t,value(i,:),linetype,'Color',[c(i,:) linetrans],'LineWidth',linewidth);
end
hl1 = legend('$\dot{x}$', '$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$');
set(hl1,'Interpreter','latex');set(hl1,'Location','Northwest');
for i=1:1:size(value,1)    
    hh=plot(timing.t(1),value(i,1),'.','Color',c(i,:),'MarkerSize',14); hh.Annotation.LegendInformation.IconDisplayStyle = 'off';
    hh=plot(timing.t(end),value(i,end),'*','Color',c(i,:),'MarkerSize',6); hh.Annotation.LegendInformation.IconDisplayStyle = 'off';
end
title('Control','interpreter','latex','FontSize',14)
xlabel('Time (s)','interpreter','latex','FontSize',14);
ylabel('Control ($\mbox{ms}^{-1}/\mbox{rads}^{-1}$)','interpreter','latex','FontSize',14);
ax = gca;
contlim = ax.YLim;
tlim    = ax.XLim;
hold off

%**************************************************************************
% Feature Error
%**************************************************************************
figure(4)
hold on
c=hsv(size(references.features.position,1));
%temperror   = reshape([robot.error.actual],size([robot.error.actual],1),[]);
temperror = {robot.error.actual};
templength = cellfun(@(x) length(x), temperror, 'UniformOutput', false);                        % Find maximum feature vector length
for i=1:1:max(cell2mat(templength))/2
    tempfeatures = cellfun(@(x) padarray(x,[max(cell2mat(templength))-length(x)],0,'post'),...  % Pad each feature vector with 0's
        temperror, 'UniformOutput', false); 
    tempfeatures = cellfun(@(x) x(2*i-1:2*i,1)', tempfeatures, 'UniformOutput', false);         % Extarct each feature pair (from cell structure)
    rawfeatures(:,:,i) = vertcat(cell2mat(tempfeatures(:)));                                    % Convert each feature pair (to matrix structure)
    
    % Interpolate features for visualisation purposes
    tempfeatures = rawfeatures(:,:,i);          % Copy feature vector for manipulation    
    tempfeatures(find(~tempfeatures )) = NaN;   % Replace 0's with NaN's
    nanfeatures(:,:,i) = tempfeatures;          % Store feature vector with NaN's
    %tempfeatures = fixgaps(tempfeatures);       % Interpolate by ignoring NaN's
    apxfeatures(:,:,i) = tempfeatures;          % Store feature vector with interpolated values (no NaN's)
    if strcmp(imaging.type,'SPH')
        apxfeatures(:,:,i) = apxfeatures(:,:,i)*180/pi;
        nanfeatures(:,:,i) = nanfeatures(:,:,i)*180/pi;
    end
end
sumtemperror = sum(sum(abs(rawfeatures),3),2);

for i=1:1:max(cell2mat(templength))/2
    % Plot Separated Lines (uses NaN's so line is discontinuous)
    plot(timing.t(1),nanfeatures(1,1,i),'.','Color',c(i,:),'MarkerSize',14);
    plot(timing.t(end),nanfeatures(end,1,i),'*','Color',c(i,:),'MarkerSize',6);
    plot(timing.t,nanfeatures(:,1,i),linetype,'Color',c(i,:),'LineWidth',linewidth);
    plot(timing.t(1),nanfeatures(1,2,i),'.','Color',horzcat(c(i,:), 0.5),'MarkerSize',14);
    plot(timing.t(end),nanfeatures(end,2,i),'*','Color',horzcat(c(i,:), 0.5),'MarkerSize',6);
    plot(timing.t,nanfeatures(:,2,i),linetype,'Color',horzcat(c(i,:), 0.3),'LineWidth',linewidth);
end
title('Image Feature Error','interpreter','latex','FontSize',14)
xlabel('Time (s)','interpreter','latex','FontSize',14);
if strcmp(imaging.type,'SPH')
    ylabel('Error (deg)','interpreter','latex','FontSize',14);
elseif strcmp(imaging.type,'CRT')
    ylabel('Error (pix)','interpreter','latex','FontSize',14);
elseif strcmp(imaging.type,'POL')
    ylabel('Error (deg/pix)','interpreter','latex','FontSize',14);
else
    ylabel('Error (deg/pix/m)','interpreter','latex','FontSize',14);
end
hold off

hold off

%**************************************************************************
% 
%**************************************************************************
figure(5)
hold on
%roboterror = temperror(k,:);
plot(timing.t(1),sumtemperror(1)*180/pi,'.','Color','k','MarkerSize',14);
plot(timing.t(end),sumtemperror(end)*180/pi,'*','Color','k','MarkerSize',6);
plot(timing.t,sumtemperror'*180/pi,linetype,'Color','k','LineWidth',linewidth);
title('Total Image Feature Error','interpreter','latex','FontSize',14)
xlabel('Time (s)','interpreter','latex','FontSize',14);
ylabel('Sum Error (deg/pix)','interpreter','latex','FontSize',14);
hold off


disp(horzcat('Total Image Feature Error (final time instant): ',num2str(sumtemperror(end)*180/pi),' (deg/pix)'));  

%**************************************************************************
% Target Range
%**************************************************************************
figure(6)
hold on
%roboterror = temperror(k,:);
plot(timing.t(1),robot.actual.range(1),'.','Color','k','MarkerSize',14);
plot(timing.t(end),robot.actual.range(end-1),'*','Color','k','MarkerSize',6);
plot(timing.t,robot.actual.range(1:end-1)',linetype,'Color','k','LineWidth',linewidth);
plot(timing.t,ones(size(timing.t)).*references.range,':','Color','k')
title('Target Range','interpreter','latex','FontSize',14)
xlabel('Time (s)','interpreter','latex','FontSize',14);
ylabel('Range (m)','interpreter','latex','FontSize',14);
hold off



%% Animations
if strcmp(animations,'ON')

    %**********************************************************************
    % Image Features
    %**********************************************************************
    %vf = VideoWriter('features.mp4','MPEG-4');
    %open(vf)
    
    f=figure(100);hold on
    f.Color = 'w';
    axis([ft1lim ft2lim]);
    xlabel(xlab,'interpreter','latex','FontSize',14);
    ylabel(ylab,'interpreter','latex','FontSize',14);
    title('Image Features','interpreter','latex','FontSize',14)

    c=hsv(size(references.features.position,1));
    value = reshape([robot.actual.features(:).position],4,2,timing.N);
    if strcmp(imaging.type, 'SPH')
        value = value*180/pi;
        referencesposition = references.features.position*180/pi;
    end
    for k=1:1:size(value,3) 
        for i=1:1:size(references.features.position,1)    
            if k==1
                plot(referencesposition(i,2),referencesposition(i,1),'o','Color',c(i,:),'MarkerSize',10);
                plot(value(i,2,1) ,value(i,1,1),'.','Color',c(i,:),'MarkerSize',14);          
            elseif k==size(value,3)
                plot(value(i,2,end) ,value(i,1,end),'*','Color',c(i,:),'MarkerSize',6);
            else
                plot(reshape(value(i,2,1:k),[],1) ,reshape(value(i,1,1:k),[],1),'-','Color',[c(i,:) linetrans],'LineWidth',2);
            end    
        end
        drawnow
        %B = getframe(gcf);
        %writeVideo(vf,B)
    end
    %close(vf)


    %**************************************************************************
    % Control
    %**************************************************************************
    %vc = VideoWriter('control.mp4','MPEG-4');
    %open(vc)
    
    g=figure(200);          
    hold on; g.Color = 'w';
    value = reshape([robot.control(:).actual],6,[]);
    c = [1 0 0;0 1 0;0 0 1;1 0 1;0 0 0;0 1 1];
    for i=1:1:size(value,1)    
        plot(timing.t(1),value(i,1),'.','Color',c(i,:),'MarkerSize',14);  
        hh(i) = plot(NaN,NaN,'.','Color',c(i,:),'MarkerSize',14);   
    end
    axis([tlim contlim]);
    hl1=legend(hh,'$\dot{x}$', '$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$');
    set(hl1,'Interpreter','latex');set(hl1,'Location','Northeast');
    title('Control','interpreter','latex','FontSize',14)
    xlabel('Time (s)','interpreter','latex','FontSize',14);
    ylabel('Control ($\mbox{ms}^{-1}/\mbox{rads}^{-1}$)','interpreter','latex','FontSize',14);

    for k=1:1:size(value,2)
        for i=1:1:size(value,1)    
            h1=plot(timing.t(1:k),value(i,1:k),'-','Color',[c(i,:) linetrans],'LineWidth',2);
            h1.Annotation.LegendInformation.IconDisplayStyle = 'off';
            if k==size(value,2)
                h2=plot(timing.t(end),value(i,end),'*','Color',c(i,:),'MarkerSize',6);
                h2.Annotation.LegendInformation.IconDisplayStyle = 'off';
            end
        end
        drawnow
        %D = getframe(gcf);
        %writeVideo(vc,D)
    end
    hold off
    %close(vc)

end

