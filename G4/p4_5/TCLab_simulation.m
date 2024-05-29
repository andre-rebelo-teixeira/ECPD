% Simulation of the TCLab linear model previously identified
%
% Loads the model identified in the TCLab_identification script, creates
% the h1 and T1C functions that mimick the TCLab interface, and performs a
% simulation starting at ambient temperature.
% You will be developing and testing your MPC controller and Kalman filter
% in this simulation environment. 
%
% Afonso Botelho and J. Miranda Lemos, IST, May 2023
%__________________________________________________________________________

% Initialization
clear all
close all
clc

% Load model
load('singleheater_model.mat','A','B','C',['' ...
    'Ke'],'e_var','y_ss','u_ss','Ts');
n = size(A,1);
e_std = sqrt(e_var); % input disturbance standard deviation
e_std = 0;
% Build the functions for applying the control and reading the temperature,
% mimicking the TCLab interface
x_ss = [eye(n)-A; C]\[B*u_ss; y_ss];
c1 = ((eye(n)-A)*x_ss - B*u_ss) * 1.1;
c2 = (y_ss - C*x_ss);
h1 = @(x,u) A*x + B*u + Ke*e_std*randn + c1; % apply control
T1C = @(x) C*x + e_std*randn + c2; % read temperature

% Simulation parameters
T = 4000; % Experiment duration [s]
N = T/Ts; % Number of samples to collect

% Open-loop control profile
u = zeros(1,N);
u(:,1:200)   = u_ss;
u(:,201:400) = u_ss+5;
u(:,401:600) = u_ss-5;
u(:,601:800) = u_ss;

% Initial conditions (start at ambient temperature, i.e. equilibrium for u = 0)
Dx0Dy0 = [eye(n)-A, zeros(n,1); C, -1]\[-B*u_ss; 0];
Dx0 = Dx0Dy0(1:n);

% Initialize signals
t = nan(1,N);
x = nan(n,N);
y = nan(1,N);
Dy = nan(1,N);
Du = nan(1,N);
Du_bar = nan(1,N);
Dx = nan(n,N+1);
x(:,1) = Dx0 + x_ss;

Dr = 2;

dx_hat = nan(n , N+1);
du_hat = nan(1, n);

Dx_bar = C \ Dr;
Du_bar = B\ (eye(n) - A) * Dx_bar;

% variables initialized for the porpuse of exercise 4 
u0_next = nan(1, N);

% Mpc horiont definition
H = 50; 
R = 0.05;  % the smaller the value, the more it will want to have negative actuations


MAX_TEMP = 50;
% Simulate incremental model
fprintf('Running simulation...');

for k = 1:N
    % Computes analog time
    t(k) = (k-1)*Ts;

    % Reads the sensor temperature
    y(:,k) = T1C(x(:,k));

    % Compute incremental variables
    Dy(:,k) = y(:,k) - y_ss;
    Dx(:,k) = x(:,k) - x_ss;
    % Du(:,k) = u(:,k) - u_ss;
    dx_hat(:, k) = Dx(:, k) - Dx_bar;

    u(:,k) = mpc_solve(dx_hat(:, k), u_ss, Du_bar, Dr, H, R, A, B, C,MAX_TEMP, y_ss);
    if  u(:, k) == -1
        u(:, k) = 0;
    else 
        Du(:, k)   = u(:, k) + Du_bar;
        u(:, k) = u(:, k) + u_ss;
    end
    % Applies the control variable to the plant
    x(:,k+1) = h1(x(:,k),u(:,k));
end
fprintf(' Done.\n');

%% Plots
% Plot absolute variables
figure('Units', 'normalized', 'Position', [0.2 0.5 0.3 0.4])
subplot(2, 1, 1), hold on, grid on   
title('Absolute input/output')
plot(t, y, '.', 'MarkerSize', 5)
yl = yline(y_ss, 'k--');
xlabel('Time [s]')
ylabel('y [°C]')
legend(yl, '$\bar{y}$', 'Interpreter', 'latex', 'Location', 'best')
subplot(2, 1, 2), hold on, grid on   
stairs(t, u, 'LineWidth', 2)
yl = yline(u_ss, 'k--');
yline(0, 'r--')
yline(100, 'r--')
xlabel('Time [s]')
ylabel('u [%]')
legend(yl, '$\bar{u}$', 'Interpreter', 'latex', 'Location', 'best');

% Plot incremental variables
figure('Units', 'normalized', 'Position', [0.5 0.5 0.3 0.4])
subplot(2, 1, 1), hold on, grid on   
title('Incremental input/output')
plot(t, Dy, '.', 'MarkerSize', 5)
xlabel('Time [s]')
ylabel('\Delta{y} [°C]')
subplot(2, 1, 2), hold on, grid on   
stairs(t, Du, 'LineWidth', 2)
yline(-u_ss, 'r--')
yline(100 - u_ss, 'r--')
xlabel('Time [s]')
ylabel('\Delta{u} [%]')

%--------------------------------------------------------------------------
% End of File