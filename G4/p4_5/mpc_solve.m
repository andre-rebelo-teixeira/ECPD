function u0 = mpc_solve(x0, u_ss, Du_bar, Dr, H, R, A , B, C, MAX_TEMP, y_ss)
    %% dense formulation
    W = compute_W(A, B, C, H);
    M = compute_m(W, H, R); 
    PI_ = compute_PI_(A, C, H); 

    Rs = [R * eye(H) zeros(H, H); zeros(H, H) 10 * eye(H)];
    Ws = [W zeros(H, H)];
    Ms = Ws' * Ws + Rs;

    % calculate both the values of F and f for the quadprog function
    F = 2 * M;
    f = 2 * x0' * PI_' * W;

    % actuation limits between 0 ->  100 %
    lb = [zeros(H, 1) - u_ss - Du_bar, zeros(H, 1)]; 
    ub = [zeros(H, 1) + 100 - u_ss - Du_bar, inf(H,1)];


    % Safety constraint to have the temperature less or equal than 55º
    % A*x <  B

    G = eye(H);
    g = (MAX_TEMP - y_ss - Dr) * ones(H, 1);
    A_ = [G * W, -eye(H)];
    B_ = g - G * PI_ * x0;
    
    options = optimoptions(@quadprog, 'MaxIterations', 2000, 'Display', 'off');
    [U, ~, exitflag, ~, ~] = quadprog(2*Ms,  (2 * x0' * PI_' * Ws), A_, B_, [], [],  lb, ub, x0, options);
   
    if exitflag ~= 1
        fprintf("Something went wrong, the exit  flag was this %d\n", exitflag); 
        fprintf("For more help check here: https://www.mathworks.com/help/optim/ug/quadprog.html#mw_bd42ef06-6096-4303-afaa-7b3cb9c539b6");
        u0 = -1;
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

    W_ = zeros(H, H);

    for ii = 1:H
        for jj = 1:H
            if jj > ii
                continue
            end
            W_(ii, jj) = C * A^(ii - jj) * b; 
        end 
    end

    W = W_;
end

%% Simple function to compute the M matrix for the system
function M = compute_m(W, H, R) 
    M = W' * W + R * eye(H);
end 

%% Simple function to compute PI matrix for the system
function PI_ = compute_PI_(A, C, H) 
    PI_ = zeros(H,size(A,1));

    for ii = 1:H
        PI_(ii, :) = C * A ^ii;
    end

end