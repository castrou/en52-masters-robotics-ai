function [output] = enn584_student_SLAM()  
%close all
%figure(1)
%grid on; axis equal; 
%axis([-2 5 -2 5]);
%hold on
 
    load_data();  
    mu = [0;0;0*pi/180];
    Sigma = diag([0.1 0.1 0.1*pi/180]).^2; 
    grid on; axis equal;
    axis([-2 5 -2 5]);
    hold on     
            % this simulator runs for 50 steps
    nsteps = 100;
    % the map is not given, plotting is just to tell us how
    % accurate is our solution 
             
    % the covariance of the process and measurements noise.
    sigma_z = 0.5;
    sigma_angle = 2;
    sigma_dist = 0.5;
    sigma_theta = 5;
    R = diag([sigma_z sigma_angle*pi/180]).^2;
    Q = diag([sigma_dist, sigma_theta*pi/180]).^2;
    % 

    for k = 1:nsteps
        % the true pose is given
        %pause(0.5)
        %clf
        %grid on; axis equal; 
        %axis([-2 5 -2 5]);
        %xr          = ask_the_oracle(k);    
        %plot_robot(xr)  
        
        %
        %
        [d,dth]  = get_odom(k);    
        [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);

        % The sensor measurements to all the landmarks at the 
        % current time step. z is a matrix of shape nx2 
        % where n is the number of landmarks in the map. 
        % The first column in z is the range (m) and
        % the second column is the bearing (rad).
        z          = sense(k);
        
        if k == 1
            % We use the sensor readings from the first time step 
            % to initialise all the landmarks in the map. 
            % this function takes:
            %   z : The sensor measurements to all the landmarks 
            %        at the current time step.
            %   Q: the covariance of the measurements
            %   
            % The function returns mu and Sigma after initializing
            % all t he landmarks
            [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);             
        else                    
           for i=1:length(z)
                % i is the id of a landmark
                % zi  is the range and bearing to landmark i
                zi     = z(i,:);
                
                % this function takes:
                %   i: id of landmark
                %   zi: the range and nearing to landmark i
                %   Q: the covariance of the measurements
                %   mu,Sigma: the current estimate of the map.
                %   
                % The function returns mu and Sigma after performing
                % an update step using the sensor readings of
                % landmark i                        
                [mu, Sigma] = update_step(i,zi,Q,mu,Sigma);                        

           end
           plot_robot(mu);
            plot_cov(mu,Sigma,3,'b');
            plot_map();
            for i=1:length(z)
                lidx = 3+2*i;
                li = mu(lidx-1:lidx);
                scatter(li(1),li(2),'b+');
                lSigma = Sigma(lidx-1:lidx,lidx-1:lidx);
                plot_cov(mu(lidx-1:lidx),lSigma,3,'g');
            end

        end   
        pause(0.1)
    end            
    output = [mu,Sigma];
                
end    

% ---------------
function [mu,Sigma] =predict_step(mu,Sigma,d,dth,R)
    th         = mu(3);
    mu          = move(mu,d,dth);
    %%% YOUR CODE HERE. CALCULATE THE TWO JACOBIANS TO IMPLEMENT THE
    %%% PREDICT STEP IN EKF SLAM
    Odom_Jacobian = [1 0 -(d)*sin(th);
                    0 1 (d)*cos(th);
                    0 0 1];
    Sensor_Jacobian = [cos(th) 0;
                        sin(th) 0;
                        0 1];
    s = size(Sigma,1);
    if s == 3
        Sigma = Odom_Jacobian*Sigma*Odom_Jacobian'+Sensor_Jacobian*R*Sensor_Jacobian';
    else
        nn = s - 3;
        j1 = [Odom_Jacobian zeros(3,nn); zeros(nn,3) eye(nn)];
        j2 = [Sensor_Jacobian;zeros(nn,2)];
        Sigma = j1*Sigma*j1'+j2*R*j2';
    end
end

function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)
    for i=1:length(z)
        n      = length(mu);
        zi     = z(i,:);
        l      = getLandmark(zi,mu(1:3));
        mu     = [mu ; l];
        L      = getLJac(zi,mu(1:3));
        lSigma = L*Q*L';
        Sigma  = [Sigma zeros(n,2);zeros(2,n) lSigma];        
    end
end

function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma)
    n      = length(mu) - 3;   
    l      = mu(3+2*landmarkID-1:3+2*landmarkID);
    [Giz , Giv]     =  jacobiansZ(l,mu(1:3));
    G      = zeros(2,3+n);
    lidx   = landmarkID*2 -1;
    G(:,3+lidx:3+lidx+1) = Giz;   
    G(:,1:3)                     = Giv;
    zp = predicZ(mu(4:end),mu(1:3),lidx);
    r = zi' - zp;
    r(2) = wrapToPi(r(2));

    %%%YOUR CODE HERE. USING G, Sigma, Q, r, mu, determine K to update mu
    %%%and Sigma

    %%%Inputs: G, Sigma, Q, r, mu
    
    %%% Outputs: mu, Sigma (via K, the Kalman gain)
    
    %Calculate Kalman gain
    K = Sigma*G'/(G*Sigma*G' + Q);

    %Update mu
    mu = mu + K*r;  
    %Update Sigma
    Sigma = (eye(length(mu)) - K*G) * Sigma;
    Sigma = 0.5*(Sigma + Sigma');
end

function xt = move(xt,d,dth)
    th  = xt(3);
    xt(1)  = xt(1) + d * cos(th);
    xt(2)  = xt(2) + d * sin(th);
    xt(3)  = xt(3) + dth;   
    xt(3) = wrapToPi(xt(3));
end
% -----------Add your functions below this line and use them in the main loop above---

 function [j1,j2] = jacobiansP(d,th)
    j1 = [ 1, 0, -d*sin(th);
       0, 1,  d*cos(th);
       0, 0,          1];
   
    j2 = [ cos(th), 0;
       sin(th), 0;
       0, 1];
 end
    
function [Gz,Gv] = jacobiansZ(l,x)
    d = l - x(1:2);
    r = norm(d);
    Gz = [d(1)/r d(2)/r;
         -d(2)/(r^2) d(1)/r^2];
    Gv = [-d(1)/r -d(2)/r 0;
             d(2)/(r^2) -d(1)/r^2 -1
             ];
end

function L = getLJac(zi,xr)
L = [cos(xr(3)+zi(2))  -zi(1)*sin(xr(3)+zi(2));sin(xr(3)+zi(2))  zi(1)*cos(xr(3)+zi(2))];
end

function l = getLandmark(zi,xr)
   l = xr(1:2) + [zi(1)*cos(zi(2)+xr(3));zi(1)*sin(zi(2)+xr(3))];
end

function z = predicZ(map,xr,i)
        z = [0 0]';
        zx = map(i);
        zy = map(i+1);
        rx = xr(1);
        ry = xr(2);
        rth = xr(3);
        r = sqrt((rx-zx)^2 + (ry-zy)^2);
        b = atan2(zy-ry,zx-rx) - rth;        
        z(1) = r ;
        z(2) = b;
        z(2) = wrapToPi(z(2));
end