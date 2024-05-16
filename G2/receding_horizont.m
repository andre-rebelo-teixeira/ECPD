%% P1 
% x(t+1) = 1.2 x(t) + u(t)

R = 1;
A = 1.2;
B = 1; 
Q = 1; 

[K, S, CLP] =  dlqr (A, B, Q, R);

%% P2 
% compute the optimzat H  from different values of H

% set the length of the horizont
H = 10; 

C = 1;
