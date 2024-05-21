%% Initialization
clear allz 
close all
clc
tclab;

%% Data collection
led(1); % make the hot led on to reduce chances of accidents

% Defininng simulation parameters
T = 1000; % experiment duration [s]
Ts = 1; % sampling period [s]
N = T/Ts; % number of samples to collect

% variables that will be used to store data
temperatures_vector = zeros(2, N);
duty_cycles_vector = zeros(2, N);

% choose the heater you want to use either 1 or 2
heater_choosen = 1;

% number of duty cycles we will itterate by 
number_of_changes_in_duty = 10; 

% this makessure we never turn on full power so save the hardware
max_duty = 60;

% create random array for the duty cycle values
duty_cycle = randperm(max_duty, number_of_changes_in_duty) + 10

% create a vector of time to choose how long we will be in each duty cycle
times =  rand(1, number_of_changes_in_duty);
times = round(times * (N + 1) / sum(times))

%%

pause(10);
% auxiliar variables for data collection
itterator_dutys = 1; 
current_duty_counter = 1;



for i = 1:N
    tic;

    duty_cycles_vector(heater_choosen, i) = duty_cycle(itterator_dutys);
    
    temperatures_vector(1, i) = T1C(); 
    temperatures_vector(2, i) = T2C();

    h1(duty_cycles_vector(1, i));
    h2(duty_cycles_vector(2, i));
    
    if current_duty_counter >= times(itterator_dutys) 
        itterator_dutys = itterator_dutys + 1;
        current_duty_counter = 0;
    end
    current_duty_counter = current_duty_counter + 1;

    pause(max(0, Ts - toc));

    disp(["itteration ", i, " duty ", duty_cycle(itterator_dutys), " current_duty_counter ", current_duty_counter]);

end

h1(0); 
h2(0); 
led(0);


