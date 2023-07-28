function [ features ] = ImageFeatures(imaging, robotpose, target)

% Author:       Aaron McFadyen
% Date:         May 2023
% Details:      Calculates image features given relative position between 
%               platform/camera and reference target.

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Image Features
%**************************************************************************
% Returns matrix where each row is an image feature and each columns
% are the components of that features. Point features (s1,..sN) are assumed 
% each with two compenents (a,b) such that 
% [   s1a s1b
%     s2a s2b
%     s3a s3b
%     s4a s4b ...]

for i=1:1:size(target.position,1)
    switch imaging.type
        case 'CRT'      % Cartesian Camera Model
            u = 1; 
            v = 1;
            r = 1;
            features.position(i,:) = [u v];
            features.range(i,1) = r;
            error('CARTESIAN coorindates currently unsupported.')

        case 'SPH'      % Spherical Camera Model
            % Find Target Position in Camera Frame (Camera Frame ~ Body Frame)
            T = body2ned(robotpose(4),robotpose(5),robotpose(6));
            cameraframe_position = T'*(target.position(i,:) - robotpose(1:3))';                 
            r = norm(cameraframe_position); 	% Range (3D)

            % Find Spherical Image Features 
            gamma = atan2(cameraframe_position(2),cameraframe_position(1));     % Azimuth    = gamma ~ phi   (corke's work)
            sigma = acos(cameraframe_position(3)/r);                            % Colatitude = sigma ~ theta (corke's work)
            features.position(i,:) = [sigma gamma];
            features.range(i,1) = r;

            % Cartesian Coordinates (Unit Sphere)
            x = cameraframe_position(1)/r;
            y = cameraframe_position(2)/r;
            z = cameraframe_position(3)/r;                   
            features.unit(i,:) = [x y z]; 

        case 'POL'      % Polar Camera Model            
            rho     = 1;
            theta   = 1;
            r       = 1;
            features.position(i,:) = [rho theta];
            features.range(i,1) = r;
            error('POLAR coorindates currently unsupported.')

        otherwise
           error('Incorrect camera model selected! Select camera TYPE to CRT (Cartesian), SPH (Spherical) or POL (Polar)'); 
    end
end

end

