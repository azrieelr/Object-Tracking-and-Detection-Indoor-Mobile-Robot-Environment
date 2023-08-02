clc; clear all;

%% Load data
load('FileLidar.mat'); % Replace with your own data file
dt = 0.05; % Time step

%% Set up EnKF parameters
N = 50; % Number of ensemble members
nx = 6; % State vector size
R = diag([0.2^2 0.2^2]); % Observation error covariance

%% Initialize EnKF
x = zeros(nx, N); % Ensemble of state vectors
for i = 1:N
    x(:, i) = [0.1*(rand()-0.5); 0.1*(rand()-0.5); 0; 0; 0; 0];
end
x_mean = mean(x, 2); % Ensemble mean
x_cov = cov(x'); % Ensemble covariance

%% Define state transition function
f = @(x) [x(1)+x(3)*dt*cos(x(5)); x(2)+x(3)*dt*sin(x(5)); x(3); x(4); x(5)+x(6)*dt; x(6)];

%% Define observation function
h = @(x) [x(1); x(2); x(3)];

%% Time loop
for t = 1:size(scans, 2)
%for t = 1:min(size(hasilLidar, 2), N)
    if t > size(scans, 2)
        break;
    end
    %% Propagate the ensemble
    for i = 1:N
        x(:, i) = f(x(:, i)) + [0.05*(randn()); 0.05*(randn()); 0.1*(randn()); 0.1*(randn()); 0.1*(randn()); 0.1*(randn())];
    end

    %% Compute ensemble mean and covariance
    x_mean = mean(x, 2);
    x_cov = cov(x');

    %% Update the ensemble
    for i = 1:N
        %y = hasilLidar(:, t) + [0.1*(randn()); 0.1*(randn())]; % Add Gaussian noise to the observation
        y = scans(:,:,t) + 0.1*randn(size(scans(:,:,t)));
        H = jacobianest(h, x(:, i)); % Approximate observation Jacobian using numerical differentiation
        K = x_cov * H' / (H * x_cov * H' + R); % Kalman gain
        x(:, i) = x(:, i) + K'*(y(:, i) - h(x(:, i)));
    end

    %% Resample the ensemble
    w = mvnpdf(x', x_mean', x_cov); % Compute likelihood for each member
    w = w / sum(w);
    x = datasample(x', N, 'Weights', w)'; % Resample the ensemble

    %% Estimate and predict
    x_mean = mean(x, 2);
    x_cov = cov(x');
    estimation(:, t) = x_mean;
    prediction(:, t) = f(x_mean);
end

%% Plot results
figure
plot(scans(1, :), scans(2, :), 'o')
hold on
plot(estimation(1, :), estimation(2, :), 'x')
plot(prediction(1, :), prediction(2, :), 'LineWidth', 2)
legend('Observed', 'Estimated', 'Predicted')
xlabel('x')
ylabel('y')
title('EnKF for Dynamic Obstacle Estimation')