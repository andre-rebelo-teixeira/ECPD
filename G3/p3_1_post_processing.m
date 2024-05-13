clear; clc; close all; 

load openloop_data_1.mat;
figure(1); 
subplot(2, 1, 1); 
hold on; grid on; 

title("Temperature evolution over time of simulation", "FontSize", 12); 
xlabel("Time in Second (s)"); 
ylabel("Temperature in Celcius (ºC)");
xlim([-10, 8010]);
ylim([-10, 110]);

%indices to ignore last value
indices = 1:(length(temps)-1);
plot(2 * indices, temps(1, indices), "Color", "red", "LineStyle", "--","LineWidth", 2, "Marker","*", "MarkerSize", 2);
plot(2 * indices, temps(2, indices), "Color", "blue", "LineStyle", "--","LineWidth", 2, "Marker","*", "MarkerSize", 2);

legend("Temperature 1", "Temperature 2");

subplot(2, 1, 2);
hold on; grid on; 
title("Duty Cycle request over time of simulation", "FontSize", 12); 
xlabel("Time in Second (s)"); 
ylabel("Duty Cycle in percentage (%)");
xlim([-10, 8010]);
ylim([-10, 110]);


%indices to ignore last value
indices = 1:(length(dutys)-1);
plot(2 * indices, dutys(1, indices), "Color", "red", "LineStyle", "--","LineWidth", 2, "Marker","*", "MarkerSize", 2);
plot(2 * indices, dutys(2, indices), "Color", "blue", "LineStyle", "--","LineWidth", 2, "Marker","*", "MarkerSize", 2);

legend("Duty Cycle 1", "Duty Cycle 2");