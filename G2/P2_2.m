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
H = 10; % horizont size
C = sqrt(diag(Q));


K_RH = zeros(1, H);
K_LQ = zeros(1, H);

for i = 1:H
    K_LQ(i) = dlqr(A, B, Q, R);

    K_RH(i) = calculate_RH_gain(A, B, C, i, R);
end

function K_RH = calculate_RH_gain(A, b, C, H, R) 
% lower triangular matrix of value b
    W =  tril(zeros(H, H) + b) * C;

    for i = 1:H
        vec = [zeros(1, i-1), A.^(1:(H-i+1))];
        W(:, i) = W(:, i) .* vec';
    end

    PI_ = zeros(1, H) + C;
    PI_ = (PI_ .* A.^(1:H))';

    e1  = zeros(1, H);
    e1(1) = 1;

    M = W' * W  + R * eye(H);
    K_RH = e1 * M^-1 * W' * PI_;
end