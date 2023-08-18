clear;
close all;

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
R = 10 * eye(2)
Q = [sig_x, 0, 0, 0;
     0, sig_y, 0, 0;
     0, 0, sig_dx, 0;
     0, 0, 0, sig_dy;];
H = [eye(2), zeros(2)];

x = zeros(T,4);
y = zeros(T,2);
x_0 = [10000, 10000, 1, 0]';
x_hat = zeros(T,4);
x_hat(1, :) = x_0 + [100, 100, 0.1, -0.1]';

P_next = 1000 * eye(4);

x, y = noisy_meas_seq(T, x_0, x_0, A, H, Q, R);

for k = 2:T
    P_prev = P_next;
    x_pred, P_pred = kalman_predict(A, x_hat(k-1, :), P_prev);
    z_k = y(k, :);
    x_hat(k, :), P_next = kalman_measure(x_pred, P_pred, z_k, H);
end

%% Data Generation
function x_truth, y_meas = noisy_meas_seq(len, x_0, y_0, A, H, Q, R)
    x_truth = zeros(len,4);
    y_meas = zeros(len,2);
    x_truth(1, :) = x_0;
    y_meas(1, :) = y_0;
    for k = 2:len
        x_truth(k, :) = A .* x_truth(k-1, :) + Q*randn([4 1]);
        y_meas(k, :) = H .* x_truth(k, :) + R*randn([2 1]);
    end
end

%% Kalman Time Update
function x_pred, P_pred = kalman_predict(A_prev, x_prev, P_prev, Q_prev)
    x_pred = A_prev.*x_prev;
    P_pred = A_prev.*P_prev.*A_prev' + Q_prev;
end

%% Kalman Meas Update
function x_est, P = kalman_measure(x_pred, P_pred, z, H)
    K = P_pred.*H'.*inv(H.*P_pred.*H'+R);
    x_est = x_pred + K.*(z-H.*x_pred);
    P = (eye(4) - K.*H).*P_pred;
end

%% Compact Representation Kalman Update
function x_hat_k, P_next_pred = kalman_update(A_prev, x_hat_prev, y, P_pred, R, Q, H)
    K = inv(P_pred .* H' .* (H.*P_pred.*H' + R));
    x_hat = A_prev .* x_hat_prev + K.*(y - H.*A_prev.*x_hat_prev);
    P_next_pred = A .* (P_pred - K.*H.*P_pred).*A' + Q;
end