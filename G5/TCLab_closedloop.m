% Closed-loop experiment for data collection in TCLab
%
% Copy of TCLab_openloop with some placeholder suggestions for where to
% place controller and state observer.
%
% If you see the warning 'Computation time exceeded sampling time by x
% seconds at sample k', it is because the computation time in a given
% loop took more than the sampling period Ts. Try to disable the rt_plot
% flag to fix it.
%
% Functions called: tclab.
%
% J. Miranda Lemos and Afonso Botelho, IST, May 2023
%__________________________________________________________________________

% Initialization
clear all %#ok<CLALL> 
close all
clc
tclab;

% Load model
load('singleheater_model.mat','A','B','C','Ke','e_var','y_ss','u_ss','Ts');
n = size(A,1);

% Experiment parameters
T = 2000; % experiment duration [s]
N = T/Ts; % number of samples to collect
MAX_TEMP = 55;

% Kalman filter design
% TO DO: Compute augmented matrices Ad, Bd, Cd
% TO DO: L = dlqe(...);
% Kalman filter design
e_std = sqrt(e_var); % Input disturbance standard deviation
Qe = Ke * e_var * Ke';
Re = e_var;
de = 2 * e_std;

% augmented matrices
Ad = [A B; zeros(1, n) 1];
Bd = [B; 0];
Cd = [C 0];
Qed = [[Qe zeros(n, 1)]; zeros(1, n) de];

% Kalman gain
L = dlqe(Ad, eye(n + 1), Cd, Qed, Re);

% MPC parameters
H = 50;
R = 0.01;

% Initial conditions (start at ambient temperature, i.e. equilibrium for u = 0)
Dx0Dy0 = [eye(n)-A, zeros(n,1); C, -1]\[-B*u_ss; 0];
Dx0 = Dx0Dy0(1:n); % to initialize filter

% Reference
r = zeros(1, N);
r(1:125) = 30;
r(126:250) = 50;
r(251:375) = 40;
r(375:500) = 70;

Dr = r - y_ss;

% Real-time plot flag. If true, plots the input and measured temperature in
% real time. If false, only plots at the end of the experiment and instead
% prints the results in the command window.
rt_plot = true;

% Initialize figure and signals
if rt_plot
    figure
    drawnow;
end
t = nan(1,N);
u = zeros(1,N);
y = zeros(1,N);
Dy = nan(1,N);
Du = nan(1,N);
xd_est = nan(n+1,N);
xd_est(:,1) = [Dx0; 0]; % Kalman filter initialization

% String with date for saving results
timestr = char(datetime('now','Format','yyMMdd_HHmmSS'));

% Signals the start of the experiment by lighting the LED
led(1)
disp('Temperature test started.')

for k=1:N
    tic;

    % Computes analog time
    t(k) = (k-1)*Ts;

    % Reads the sensor temperatures
    y(1,k) = T1C();

    % Compute incremental variables
    Dy(:,k) = y(:,k) - y_ss;

    % Kalman filter correction step
    xd_est(:,k) = xd_est(:,k) + L*(Dy(:,k) - Cd*xd_est(:,k));

    % Computes the control variable to apply
    % TO DO: [...] = mpc_solve(...)
    % Compute the steady-state disturbance and associated state and control
    d_ss = xd_est(n + 1, k);
    Du_ss = (C / (eye(n) - A) * B) \ Dr(k) - d_ss;
    Dx_ss = (eye(n) - A) \ B * (Du_ss + d_ss);

    % Computes the control variable to apply
    dx = xd_est(1:n, k) - Dx_ss;
    du = mpc_solve(dx, u_ss, Du_ss, Dr(k), H, R, A, B, C, MAX_TEMP, y_ss);

    Du(:, k) = du + Du_ss;
    u(:, k) = Du(:, k) + u_ss;

    % Kalman filter prediction step
    xd_est(:,k+1) = Ad*xd_est(:,k) + Bd*Du(:,k);

    % Applies the control variable to the plant
    h1(u(1,k));

    if rt_plot
        % Plots results
        clf
        subplot(2,1,1), hold on, grid on   
        plot(t(1:k),y(1,1:k),'.','MarkerSize',10)

        stairs(t,r,'g--')
        xlabel('Time [s]')
        ylabel('y [°C]')
        subplot(2,1,2), hold on, grid on   
        stairs(t(1:k),u(1,1:k),'LineWidth',2)
        xlabel('Time [s]')
        ylabel('u [%]')
        ylim([0 100]);
        drawnow;
    else
        fprintf('t = %d, y1 = %.1f C, y2 = %.1f C, u1 = %.1f, u2 = %.1f\n',t(k),y(1,k),y(2,k),u(1,k),u(2,k)) %#ok<UNRCH> 
    end

    % Check if computation time did not exceed sampling time
    if toc > Ts
        warning('Computation time exceeded sampling time by %.2f s at sample %d.',toc-Ts,k)
    end
    % Waits for the begining of the new sampling interval
    pause(max(0,Ts-toc));
end

% Turns off both heaters at the end of the experiment
h1(0);
h2(0);

% Signals the end of the experiment by shutting off the LED
led(0)

disp('Temperature test complete.')

if ~rt_plot
    figure
    subplot(2,1,1), hold on, grid on   
    plot(t(1:k),y(1,1:k),'.','MarkerSize',10)
    stairs(t,r,'g--')
    xlabel('Time [s]')
    ylabel('y [°C]')
    subplot(2,1,2), hold on, grid on   
    stairs(t(1:k),u(1,1:k),'LineWidth',2)
    xlabel('Time [s]')
    ylabel('u [%]')
    ylim([0 100]);
end

%--------------------------------------------------------------------------

% Save figure and experiment data to file
exportgraphics(gcf,['openloop_plot_',timestr,'.png'],'Resolution',300)
save(['openloop_data_',timestr,'.mat'],'y','u','t');

%--------------------------------------------------------------------------
% End of File


