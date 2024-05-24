%% computing the optimal LQ gains
% Define the system matrices
A = 1.2;    % System matrix A
B = 1;      % System matrix B

% Define the weighting matrices Q and R for LQR
Q = 1;      % State weighting matrix
R = [0 0.1 1 10 100 1000];      % Control weighting matrix

%  Compute the optimal receding horizon for differente values of horizon H 
H = 20; % horizont size
C = sqrt(diag(Q));



K_RH = zeros(size(R, 2), H);
K_LQ = zeros(size(R, 2), H);
eigen_val_LQ = zeros(size(R, 2), H);
eigen_val_RH = zeros(size(R, 2), H);

figure(1);

title("Optimal Receding horizont gain with Horizont of 20 and multiple values of R");
xlabel("H"); 
ylabel("K"); 
hold on; 
grid on;

% Get colors for the plots
colorOrder = get(gca, 'ColorOrder');
numColors = size(colorOrder, 1);


plots = zeros(1, size(R, 2)); 
string_ = [];

% calculate RH gains and plot the Gains and the eigen values
for ii = 1:size(R, 2)
    R_ = R(ii);
    for jj  = 1:H
        K_LQ(ii, jj) = dlqr(A, B, Q, R_);
        K_RH(ii, jj) = calculate_RH_gain(A, B, C, jj, R_);

        eigen_val_LQ(ii, jj) = eig(A-B*K_LQ(ii, jj));
        eigen_val_RH(ii, jj) = eig(A-B*K_RH(ii, jj));

    end
    color = rem(ii, numColors);
    plots(ii) = plot(K_RH(ii, :), LineWidth=1, LineStyle="--", Marker="hexagram", MarkerSize=4, Tag='R_' + string(R_), Color=colorOrder(color, :), DisplayName="R = " + string(ii)); 
    plot(K_LQ(ii, :), LineStyle="-", LineWidth=2, Color=colorOrder(color, :), Tag='R_' + string(R_));
    string_ = [string_; "R = " + string(R_)];
end

legend(plots, string_(:)');


figure(2);
hold on;
grid on; 
title("Absolute value of the eigen value");
xlabel("H");
ylabel("|\lambda|")
for ii  = 1:size(R, 2)
    color = rem(ii, numColors);
    plots(ii) = plot(abs(eigen_val_RH(ii, :)), LineWidth=1, LineStyle="--", Marker="hexagram", MarkerSize=4, Tag='R_' + string(R_), Color=colorOrder(color, :), DisplayName="R = " + string(ii)); 
    plot(abs(eigen_val_LQ(ii, :)), LineStyle="-", LineWidth=2, Color=colorOrder(color, :), Tag='R_' + string(R_));
    string_ = [string_; "R = " + string(R_)];
end

legend(plots, string_(:)');
plot(ones(1, 20), LineStyle="-", LineWidth= 1, Color="black", DisplayName="Stability bondary");

%%% Function libraty starts here%% Calculate RH gain
%% Compute the optimar gain
function K_RH = calculate_RH_gain(A, b, C, H, R) 

    W = compute_W(A, b, C, H); 
    PI_ = compute_PI_(A, C, H);
    M = compute_m(W, H, R);

    e1 = zeros(1, H); 
    e1(1) = 1;

    K_RH = e1 * M^-1 * W' * PI_;
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