%% computing the optimal LQ gains
% Define the system matrices
A = 1.2;    % System matrix A
B = 1;      % System matrix B

% Define the weighting matrices Q and R for LQR
Q = 1;      % State weighting matrix
R = 1;      % Control weighting matrix

% Compute the optimal LQ state feedback gain
[K,S,lambda] = dlqr(A, B, Q, R);

%%  Compute the optimal receding horizon for differente values of horizon H 
H = 10; 
C = Q;
W =  zeros(H, H);


for i = 1:H 
       x

end