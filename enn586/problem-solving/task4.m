clear
rng(42);
task3();
close all;

%% Initialization
T = 1000; % no. time instants
dT = 1;

sig_x = 0;
sig_y = 0;
sig_dx = 0.01;
sig_dy = 0.01;
R = [10, 0;
     0, 0.001];
Q = [sig_x, 0, 0, 0;
     0, sig_y, 0, 0;
     0, 0, sig_dx, 0;
     0, 0, 0, sig_dy;]; 

A = [1, 0, dT, 0;
     0, 1, 0, dT;
     0, 0, 1, 0;
     0, 0, 0, 1];

x_0 = [10000, 10000, 1, 0];

N = 1000; % Number of particles
particles = repmat(x_0, N, 1) + [100*randn(N,1), 100*randn(N,1), 0.1*randn(N,1), -0.1*randn(N,1)];
weights = ones(N, 1)/N;

% Storage
x_pf = zeros(T, 4);
% Use data from Task 3

%% Particle Filter
for k = 1:T
    % Particle Propagation
    for i = 1:N
        particles(i,:) = (A * particles(i,:)')' + (sqrt(Q) * randn(4,1))';
    end
    
    % Weight Update
    for i = 1:N
        y_predicted = [sqrt(particles(i,1)^2 + particles(i,2)^2), atan2(particles(i,2), particles(i,1))];
        innovation = y(k,:) - y_predicted;
        % Correct the angle to be between -pi and pi
        innovation(2) = atan2(sin(innovation(2)), cos(innovation(2)));
        weights(i) = exp(-0.5 * (innovation/sqrt(R)) * innovation') ;
    end
    weights = weights / sum(weights); % Normalize weights

    % State Estimate
    x_pf(k,:) = sum(particles .* weights, 1);
    
    % Resampling
    indices = resampling(weights);
    particles = particles(indices, :);
    weights = ones(N, 1)/N;
end

err_sq = (x - x_pf).^2;
rms = sqrt(1/T * sum(err_sq))

%% Plotting
figure Name 'Particle Filter XY'
% Plot PF estimates using x_pf
plot(x(:,1), x(:,2),x_pf(:,1), x_pf(:,2));%,y(:,1).*cos(y(:,2)), y(:,1).*sin(y(:,2)));
legend('Ground Truth', 'Particle Filter Estimate', 'Measurements');
xlabel('X position');
ylabel('Y position');
title('Ground Truth and Particle Filter Estimate');
grid on;

%% Function Definition
% Data Generation
function [x_truth, y_meas] = nonlin_measurement_sequence(len, x_0, A, Q, R)
    x_truth = zeros([len 4]);
    y_meas = zeros([len 2]);
    
    x_truth(1, :) = x_0;
    y_meas(1, :) = [sqrt(x_0(1)^2 + x_0(2)^2), atan2(x_0(2),x_0(1))];
    for k = 2:len
        x_truth(k, :) = A * x_truth(k-1, :)' + sqrt(Q)*randn([4 1]);
        H = [sqrt(x_truth(k,1)^2 + x_truth(k,2)^2); atan2(x_truth(k,2),x_truth(k,1))];
        y_meas(k, :) = (H + sqrt(R)*randn([2 1]))';
    end
end

% Resampling function
function indices = resampling(weights)
    N = length(weights);
    indices = zeros(N, 1);
    C = cumsum(weights);
    u = rand/N;
    i = 1;
    for j = 1:N
        while u > C(i)
            i = i + 1;
        end
        indices(j) = i;
        u = u + 1/N;
    end
end
