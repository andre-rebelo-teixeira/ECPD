%%% Since the data was taken the correct variable names were not used
% this script is need, this will convet from the previous variable names to
% the ones used in tlc identification, as well as remove any outliers that
% might exist in the code 

%% clear previous data /
clear;
clc;

%% transform training data 
load("training_data.mat"); % load the training data

y = temps(:, 1:length(temps) - 3); % remove the last point since original code has an error
u = dutys(:, 1:length(dutys) - 3); % removethe last point since original code has an error when acquiring the last point


% no time was recorded during the simulation but the sampling time was 2s
Ts = 4;
t = (1:length(u)) * Ts;

save("openloop_data_1.mat", "y", "u", "t");

%% transform validation data
clear;

load("validation_data.mat");

y = temperatures_vector(:, 1:4:length(temperatures_vector));
u = duty_cycles_vector(:, 1:4:length(temperatures_vector));
Ts = 1;
t = (1:length(u)) * 4;

save("openloop_data_2.mat", "y","u","t");

clear; clc;