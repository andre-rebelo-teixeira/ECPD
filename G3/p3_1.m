% Open-loop experiment for data collection in TCLab
%
% Initializes TCLab, applies a sequence of open-loop controls and records
% the corresponding temperature.
%
% If you see the warning 'Computation time exceeded sampling time by x
% seconds at sample k', it is because the computation time in a given
% loop took more than the sampling period Ts. Try to disable the rt_plot
% flag to fix it or increase Ts.
%
% Functions called: tclab.
%
% J. Miranda Lemos and Afonso Botelho, IST, May 2023
%__________________________________________________________________________

% Initialization
clear allz 
close all
clc
tclab;

%%
led(1);
% Experiment parameters
T = 8000; % experiment duration [s]
Ts = 2; % sampling period [s]
N = T/Ts; % number of samples to collect
change_counter =  1000 / Ts;

dutys_ = [25 40 20 30 26 33 37 22];
iter = 1;

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
y = nan(2,N);

% String with date for saving results
timestr = char(datetime('now','Format','yyMMdd_HHmmSS'));

% Signals the start of the experiment by lighting the LED
led(1)
disp('Temperature test started.')



first_temp_reached = 0;
temp_maintained = 0; 
changes = 0;

temps = zeros(2, N);
dutys = zeros(2, N);

heater_chossen = 1;
dutys(heater_chossen, 1) = 30;

% Initialize plot

hPlot = plot(NaN, NaN);  % Create an empty plot
xlabel('Time');
ylabel('Data');
title('Real-time Data Plot');
grid on;

counter = 0;

figure(1);

multiplier = 1;

for i = 1:N 
    tic;
    counter = counter + 1;
    
    if counter == change_counter 
        counter = 0;
       % dutys(heater_chossen, i) = dutys(heater_chossen, i-1) * (1 + multiplier * randi([5000, 10000]) / 100000);
        multiplier = sign(2 * rand() - 1);
        iter = iter + 1;
    else 
        if i > 1 
            dutys(heater_chossen, i) = dutys(heater_chossen, i-1);
        end
    end
    
    dutys(heater_chossen, i) = dutys_(iter);


    temps(1, i) = T1C(); 
    temps(2, i) = T2C();

    %disp(["duty", dutys(1, i)]);
    %disp(["temps", (temps(1, i))]);
    disp(["counter", ifaçd5])
    h1(dutys(1, i));
    h2(dutys(2, i));
    
    



    pause(max(0, Ts - toc));
end

  %  subplot(2,2,1);
   % hold on;
    %grid on;
%    plot(1:i, dutys(1,1:i), Color='red', LineStyle='--', LineWidth=2, Marker='*');
%    plot(1:i, dutys(2,1:i), Color='blue', LineStyle='--', LineWidth=2, Marker='*');
%    title("Fulle Temp plot");
%    xlabel("itteration");
%    ylabel("duty cycle %");

    legend("duty cycle 1", "duty cycle 2");
    hold off;
 
    subplot(2,2,2);
    hold on;
    grid on;
    plot(1:i, temps(1,1:i), Color='red', LineStyle='--', LineWidth=2, Marker='*');
    plot(1:i, temps(2,1:i), Color='blue', LineStyle='--', LineWidth=2, Marker='*');
    legend("temp 1", "temp 2");
    title("Full duty cycle plot");
    xlabel("Itteration");
    ylabel("temp - C");
    hold off;

    subplot(2,2,3);
    cla;
    hold on;
    grid on;
    title("Zoomed title plot");
    xlabel("iterations"); 
    ylabel("Temp - C");
    plot (max(1, i-10):i, temps(1, max(1, i-10):i),Color='blue', LineStyle='--', LineWidth=2, Marker='*');
    hold off;

%%


h1(0);
h2(0); 
led(0);
