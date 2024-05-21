function u0 = mpc_solve(x0, H, R, A , B, C)
    %% dense formulation
    W = compute_W(A, B, C, H);
    M = compute_m(W, H, R); 
    PI_ = compute_PI_(A, C, H); 

    % calculate both the values of F and f for the quadprog function
    F = 2 * M;
    f = 2 * x0' * PI_' * W;

    lb = zeros(H, 1); 
    ub = zeros(H, 1) + 100;

    options = optimoptions(@quadprog, 'MaxIterations', 2000);
    [U, val, exitflag, output, lamdba] = quadprog(F, f, [], [], [], [], lb, ub, x0, options);
  
   
    if exitflag ~= 1
        fprintf("Something went wrong, the exit  flag was this %d\n", exitflag); 
        fprintf("For more help check here: https://www.mathworks.com/help/optim/ug/quadprog.html#mw_bd42ef06-6096-4303-afaa-7b3cb9c539b6");
        u0 = 0;
        return;
    end

    %% sparse formulation code 
    % finish this latter to make sure both formulations are sent
    % for now just use the dense formulation
        

    %% return value -> will depend on the formulation
    u0 = U(1);
end 

%% Simple function to compute W matrix for our system
function W = compute_W(A,b, C, H)
    W_ =  tril(zeros(H, H) + b) * C;

    for i = 1:H
        vec = [zeros(1, i-1), A.^(1:(H-i+1))];
        W_(:, i) = W_(:, i) .* vec';
    end

    W = W_;
end

%% Simple function to compute the M matrix for the system
function M = compute_m(W, H, R) 
    M = W' * W + R * eye(H);
end 

%% Simple function to compute PI matrix for the system
function PI_ = compute_PI_(A, C, H) 
    PI__ = zeros(1, H) + C;
    PI_ = (PI__ .* A.^(1:H))';
end