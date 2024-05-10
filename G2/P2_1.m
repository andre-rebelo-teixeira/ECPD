% Define the system matrices
A = 1.2;    % System matrix A
B = 1;      % System matrix B

% Define the weighting matrices Q and R for LQR
Q = 1;      % State weighting matrix
R = 1;      % Control weighting matrix

% Compute the optimal LQ state feedback gain
[K,S,lambda] = dlqr(A, B, Q, R);

disp('Optimal LQ state feedback gain:');
disp(K);