clear;
close all;

%% init
T = 1000; % no. time instants

sig_x = 0;
sig_y = 0;
sig_dx = 0.01;
sig_dy = 0.01;
R = 10 * eye(2);
Q = [sig_x, 0, 0, 0;
     0, sig_y, 0, 0;
     0, 0, sig_dx, 0;
     0, 0, 0, sig_dy;]; 

x_0 = [10000, 10000, 1, 0];
y_0 = x_0(:, 1:2);

%% Data Generation
[x, y] = nonlin_measurement_sequence(T, x_0, y_0, A, Q, R);

%% Particle Filter Setup
pf = particleFilter(@particleStateTransition,@measurementLikelihood);
initialize(pf, 1000, [2;0], 0.01*eye(2));
pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';

% Estimate
xCorrectedPF = zeros(size(x));
for k=1:size(x,1)
    % Use measurement y[k] to correct the particles for time k
    xCorrectedPF(k,:) = correct(pf,y(k, :)); % Filter updates and stores Particles[k|k], Weights[k|k]
    % The result is x[k|k]: Estimate of states at time k, utilizing
    % measurements up to time k. This estimate is the mean of all particles
    % because StateEstimationMethod was 'mean'.
    %
    % Now, predict particles at next time step. These are utilized in the
    % next correct command
    predict(pf); % Filter updates and stores Particles[k+1|k]
end

figure();
subplot(2,1,1);
plot(timeVector,x(:,1),timeVector,xCorrectedPF(:,1),timeVector,y(:));
legend('True','Particlte filter estimate','Measured')
ylim([-2.6 2.6]);
ylabel('x_1');
subplot(2,1,2);
plot(timeVector,x(:,2),timeVector,xCorrectedPF(:,2));
ylim([-3 1.5]);
xlabel('Time [s]');
ylabel('x_2');


%% Functions
%% Data Generation
function [x_truth, y_meas] = nonlin_measurement_sequence(len, x_0, y_0, A, Q, R)
    x_truth = zeros([len 4]);
    y_meas = zeros([len 2]);
    
    x_truth(1, :) = x_0;
    y_meas(1, :) = y_0;
    for k = 2:len
        x_truth(k, :) = A * x_truth(k-1, :)' + Q*randn([4 1]);
        H = [sqrt(x_truth(k,1)^2 + x_truth(k,2)^2); atan(x_truth(k,2)^2/x_truth(k,1))];
        y_meas(k, :) = H + R*randn([2 1]);
    end
end

%% State Transition Function
function particles = particleStateTransition(particles)
    [numStates, numParticles] = size(particles);
    dt = 1; % [s] Sample time
    for n=1:numParticles
        particles(:,n) = particles(:,n) + stateTransition(particles(:,n))*dt;
    end

    % Add Gaussian noise with variance 0.025 on each state variable
    processNoise = 0.025*eye(numberOfStates);
    particles = particles + processNoise * randn(size(particles));

end

%% Continuous State Function
function dxdt = stateTransition(x)
    dT = 1;
    A = [1, 0, dT, 0;
         0, 1, 0, dT;
         0, 0, 1, 0;
         0, 0, 0, 1];
    dxdt = A*x;
end

%% Measurement Likelihood Function
function likelihood = measurementLikelihood(particles,measurement)
    numMeas = 1; % Expected number of measurements
    
    % The measurement is first state. Get all measurement hypotheses from particles
    predictedMeasurement = particles(1,:);
    
    % Assume the ratio of the error between predicted and actual measurements
    % follow a Gaussian distribution with zero mean, variance 0.2
    mu = 0; % mean
    sigma = 0.2 * eye(numMeas); % variance
    
    % Use multivariate Gaussian probability density function, calculate
    % likelihood of each particle
    numParticles = size(particles,2);
    likelihood = zeros(numParticles,size(measurement));
    C = det(2*pi*sigma) ^ (-0.5);
    for kk=1:numParticles
        errorRatio = (predictedMeasurement(kk)-measurement)/predictedMeasurement(kk);
        v = errorRatio-mu;
        likelihood(kk) = C * exp(-0.5 * (v' / sigma * v) );
    end
end
