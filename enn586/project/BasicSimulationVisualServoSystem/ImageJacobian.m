function [Jout] = ImageJacobian(imaging,features,range,dof) %VERIFIED

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Calculate the image Jacobian given relative range and image
%               features. The image Jacobian relates image feature motion
%               (velocity) to platform motion (velocity).

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Image Features & Range       
%**************************************************************************
                               % SPH COORDS | POLAR COORDs | CARTESIAN COORDS 
F1 = features(1);              %   Theta    |   Theta      |    u (pix)
F2 = features(2);              %   Phi      |   r          |    v (pix)
R  = range;                    %   R        |   R          |    R

f  = imaging.f;

%**************************************************************************
% Calculate Jacobian
%**************************************************************************
switch imaging.type
    case 'CRT'      % Cartesian Camera Model -> Order [u_dot v_dot] = J*v           (Corke 2009)
        J1 = [-f/R   0       F1/R  F1*F2/f        -(f^2 + F1^2)/f   F2];
        J2 = [0     -f/R     F2/R  (f^2+F2^2)/f   -F1*F2/f         -F1];
        J  = [J1;J2];
    case 'SPH'      % Spherical Camera Model -> Order [theta_dot phi_dot] = J*v     (Corke 2010 / Mcfadyen 2012)
        J1 = [-(cos(F2)*cos(F1))/R -(cos(F1)*sin(F2))/R sin(F1)/R    sin(F2)                     -cos(F2)                     0];
        J2 = [sin(F2)/(R*sin(F1))  -cos(F2)/(R*sin(F1)) 0             (cos(F2)*cos(F1))/sin(F1)   (sin(F2)*cos(F1))/sin(F1)  -1];
        J  = [J1;J2];
    case 'POL'   	% Polar Camera Model    -> Order [r_dot theta_dot] = J*v        (Corke 2009)
        J1 = [cos(F1)/R        sin(F1)/R      -F2/R   -(1+F2^2)*sin(F1)   (1+F2^2)*cos(F1)   0];
        J2 = [-sin(F1)/(F2*R)  cos(F1)/(F2*R)  0      -cos(F1)/F2         -sin(F1)/F2        1];
        J  = [J1;J2];
           
    otherwise
        error('Invalid Jacobian type. Select SPH, POL or CRT for IMAGING.TYPE')
end

%**************************************************************************
% Define Jacobian Columns (w.r.t number features and DOF)
%**************************************************************************
if isempty(dof)
    col = [1 2 3 4 5 6];
else
    col = sort(dof);    %Sorts in ascending order to ensure output matrix is correct 
end

%**************************************************************************
% Select Jacobian Columns (w.r.t number features and DOF)
%**************************************************************************
if length(col) > 6
    Jout = 'Invalid';
    error('Invalid input for DOF.  Ensure length DOF <= 6')
    
elseif length(col) <= 6 
    for i=1:1:length(col)
        if col(i) > 6
            Jout = 'Invalid';
            error('Invalid input for DOF.  Ensure all DOF elements <= 6');
        else
            Jout(:,i) = J(:,col(i));
        end
    end
end
    

