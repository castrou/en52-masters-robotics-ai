clear;
close all;

rng(42);

%% init
T = 1000; % no. time instants
dT = 1;

A = [1, 0, dT, 0;
     0, 1, 0, dT;
     0, 0, 1, 0;
     0, 0, 0, 1];

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

x_0 = [10000, 10000, 1, 0];
y_0 = x_0(:, 1:2);
x_hat = zeros(T, 4);
x_hat(1, :) = x_0 + [100, 100, 0.1, -0.1];

P_next = 0.1 * eye(4);

%% 3.1 Data Generation
[x, y] = nonlin_measurement_sequence(T, x_0, y_0, A, Q, R);

%% 3.2 EKF
for k = 2:T
    P_prev = P_next;
    [x_pred, P_pred] = ekf_predict(A, x_hat(k-1, :), P_prev, Q);
    z_k = y(k, :);
    [x_hat(k, :), P_next] = ekf_measure(x_pred, P_pred, z_k, R);
end

err_sq = (x_hat - x).^2;
rms = sqrt(1/T * sum(err_sq))

%% Plot
figure();
plot(x(:,1),x(:,2),x_hat(:,1),x_hat(:,2),y(:,1).*cos(y(:,2)), y(:,1).*sin(y(:,2)));
% plot(x(:,1),x(:,2),'*',y(:,1).*cos(y(:,2)), y(:,1).*sin(y(:,2)));

legend('True','EKF Estimate','Measured')
ylabel('x_1');
figure()
plot(1:T,x(:,2),1:T,x_hat(:,2),1:T,y(:,1).*sin(y(:,2)));
xlabel('Time [s]');
ylabel('x_2');

%% Functions
%% %% Data Generation
function [x_truth, y_meas] = nonlin_measurement_sequence(len, x_0, y_0, A, Q, R)
    x_truth = zeros([len 4]);
    y_meas = zeros([len 2]);
    
    x_truth(1, :) = x_0;
    y_meas(1, :) = [sqrt(y_0(1)^2 + y_0(2)^2); atan2(y_0(2),y_0(1))];
    for k = 2:len
        x_truth(k, :) = A * x_truth(k-1, :)' + sqrt(Q)*randn([4 1]);
        H = [sqrt(x_truth(k,1)^2 + x_truth(k,2)^2); atan2(x_truth(k,2),x_truth(k,1))];
        y_meas(k, :) = H + sqrt(R)*randn([2 1]);
    end
end

%% %% EKF Time Update
function [x_pred, P_pred] = ekf_predict(A_prev, x_prev, P_prev, Q_prev)
    x_pred = (A_prev*x_prev')';
    P_pred = A_prev*P_prev*A_prev' + Q_prev;
end

%% %% EKF Meas Update
function [x_est, P] = ekf_measure(x_pred, P_pred, z, R)
    h = [sqrt(x_pred(1)^2 + x_pred(2)^2); atan2(x_pred(2),x_pred(1))];
    Hbar = [ x_pred(1)/sqrt(x_pred(1)^2 + x_pred(2)^2), x_pred(2)/sqrt(x_pred(1)^2 + x_pred(2)^2), 0, 0;
            -x_pred(2)/(x_pred(1)^2 + x_pred(2)^2), x_pred(2)/(x_pred(1)^2 + x_pred(2)^2), 0, 0];
    K = P_pred*Hbar'*inv(Hbar*P_pred*Hbar'+R);
    % x_est = x_pred + (K*(z'-Hbar*x_pred'))'; 
    x_est = x_pred + (K*(z'-h))'; 
    P = P_pred - K*Hbar*P_pred;
end

