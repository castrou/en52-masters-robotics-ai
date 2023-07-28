function [YourVariables,u,e ] = GetENN586ImageControl(YourVariables,imaging, references, noise, control, ENN586error, features)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Calculates control input for a variable number of images
%               features including added process noise (if selected).
%               Control approach is 'classical' visual servoing.

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Find Control Input (Classical Servoing, Variable Number Points)
%**************************************************************************

%range_error=[range_error_status, range_error_type, range_error_size]

for i=1:1:size(features.position,1)

    % if there are feature location errors)
    if strcmp(ENN586error.feature.status, 'ON')
        features.position(i,:)=features.position(i,:)+ ENN586error.feature.offset;
    end

    if strcmp(imaging.type,'SPH')
        e.actual(2*i-1:2*i,1) = wrapToPi(features.position(i,:) - references.features.position(i,:));
    elseif strcmp(imaging.type,'POL')
        % TBD - one fet. in px/mm and the other in deg/rad
        %e.actual(2*i-1:2*i,1) = wrapToPi(newfeatures.position(i,:) - references.features.position(i,:));
    else
        e.actual(2*i-1:2*i,1) = features.position(i,:) - references.features.position(i,:);
    end

%    Ja(2*i-1:2*i,:) = ImageJacobian(imaging,features.position(i,:),features.range(i),[]); %Will do perfect servoing as r = r(t)!
% if there are range errors
    if strcmp(ENN586error.range.status, 'ON')
         if strcmp( ENN586error.range.type,'BIAS')
             features.range(i)=features.range(i)+ ENN586error.range.size;
             %Ja(2*i-1:2*i,:) = ImageJacobian(imaging,features.position(i,:),features.range(i)+ ENN586error.range_size,[]);
         elseif strcmp(ENN586error.range.type, 'FACTOR')
             features.range(i)=features.range(i)* ENN586error.range.size;
             %Ja(2*i-1:2*i,:) = ImageJacobian(imaging,features.position(i,:),features.range(i)* ENN586error.range_size,[]);
         else
             error('Error with the Range error type')
        end
    else
%        Ja(2*i-1:2*i,:) = ImageJacobian(imaging,features.position(i,:),features.range(i),[]);  % no range error
    end
    Ja(2*i-1:2*i,:) = ImageJacobian(imaging,features.position(i,:),features.range(i),[]); 
end
% Apply Control For Platform/DoF
%uactual = -control.gain*pinv(Ja(:,control.dof))*e.actual;
ENN586YourControl   %return uactual, Can use "YourVariables" to store information needed in different loops


if strcmp(ENN586error.control.status, 'ON')
    uactual=uactual+ENN586error.control.disturbance;
end

switch imaging.numfeatures(end)
    case 1
        u.actual = [uactual' 0 0 0 0];
    case 2
        u.actual = [uactual(1:end-1)' 0 0 uactual(end)'];
    otherwise
        if strcmp(control.platform, 'QUAD')
            u.actual = [uactual(1:end-1)' 0 0 uactual(end)'];
        else
            u.actual = uactual';
        end                    
end

%**************************************************************************
% Add Process Noise [u + w(t)]
%**************************************************************************
if strcmp(noise.proc.on, 'ON')          
    pnoise     = normrnd(noise.proc.mu,noise.proc.std,size(u,1),size(u,2));
    u.actual   = u.actual + pnoise';    
end


