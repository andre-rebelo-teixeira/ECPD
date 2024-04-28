%% Lets first create the space where we will be working
x1min = -3;
x1max = 3;
x2min = -3;
x2max = 3;

N1 = 100;
N2 = 100;

% create the linespace
xv1 = linspace(x1min, x1max, N2);
xv2 = linspace(x2min, x2max, N2); 
[xx1, xx2] = meshgrid(xv1, xv2); 

rosenbrock_function = @(x1, x2) 100 * (x2 - x1^2)^2 + (1-x1)^2;
rosenbrock_caller = @(x) rosenbrock_function(x(1), x(2));

ff = arrayfun(rosenbrock_function, xx1 , xx2);


%% unbounded optimization 

%start point for the solver
x0 = [-1; 1];
options = optimoptions('fminunc','Algorithm','quasi-newton');
xopt=fminunc(rosenbrock_caller,x0,options);

%% bounded optimization
% x1 <= 0.5
Ac = [1 0]; 
bc = 0.5; 

xoptconst = fmincon(rosenbrock_caller, x0, Ac, bc);

%% plot 3D graph of rosenbrock function cost  

figure(1);
s = surf(xx1, xx2, ff);
set(s,'LineStyle','none')

colorbar;

% Identifies axis
gg=xlabel('x_1');
set(gg,'FontSize',14);

gg=ylabel('x_2');
set(gg,'FontSize',14);

gg=zlabel('f(x)');
set(gg,'FontSize',14);

%% Plot the countour line of the cost function
% and the minimuns

figure(2); 
hold on;

axis([x1min x1max x2min x2max]); 
axis square;
contour(xv1, xv2, ff, 300);
colorbar;

plot(xopt(1), xopt(2), 'xr');
plot(xoptconst(1), xoptconst(2), '*r');
plot(x0(1), x0(2), 'or');

legend('Countor lines', 'Uncontrained minima', 'Constrained mimina', 'initial estimate'); 
title("Rosenbrock countour lines");
xlabel("x1"); 
ylabel("y1");







