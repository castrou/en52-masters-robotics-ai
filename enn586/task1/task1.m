clear;
close all;

rng(42);

%% Set up
C = [1.3, 5].';
R = 10;
covar = (sqrt(R) * eye(2));


%% Task 1
k_count = 100
y = noisy_meas_seq(k_count, C, covar);

C_hat = zeros([k_count 2]);
C_hat(1, :) = [0.5, 1];
for k = 2:k_count
    C_hat(k, :) = estimate_step(k, C_hat(k-1,:), y(k,:))
end

C_plot = ones([k_count 2]) .* C.';
err_sq = (C_hat - C_plot) .^ 2;
rms = sqrt(1/k_count * sum(err_sq))

figure Name Measurement
subplot(1, 2, 1)
title("X val");
plot(y(:, 1))
hold on
plot(C_plot(:,1))

subplot(1, 2, 2)
title("Y val");
plot(y(:, 2))
hold on
plot(C_plot(:,2))


figure Name Estimation
subplot(1, 2, 1)
title("X val")
plot(C_hat(:, 1))   
hold on
plot(C_plot(:,1))

subplot(1, 2, 2)
title("Y val")
plot(C_hat(:, 2))
hold on
plot(C_plot(:,2))

figure Name Error
subplot(1,2,1)
plot(err_sq(:,1))
subplot(1,2,2)
plot(err_sq(:,2))

%% Function Definitions
function y_k = noisy_meas_step(C, covar)
    w_k = covar * randn([2 1]);
    y_k = C + w_k;
end

function y = noisy_meas_seq(k, C, covar)
    y = zeros([k 2]);
    for k = 1:k
        y(k, :) = noisy_meas_step(C, covar).';
    end
end

function C_hat_k = estimate_step(k, C_hat_prev, y_k)
    inv_k = 1/k;
    C_hat_k = (k-1)*inv_k*C_hat_prev + inv_k*y_k
end