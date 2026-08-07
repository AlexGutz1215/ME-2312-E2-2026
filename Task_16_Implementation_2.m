% Differential Equation Cr Bumper Problem
% Mx'' + Dx' + Kx = 0
% New initial condition

% define parameters
M = 8; % Mass - kg
D = 50; % Damping coefficient - Ns/m
K = 100; % Spring constant - N/m

% Solving using ODE45 function
% requires two first order equations
% Requires two first order equations
% x' = p/M (where p is momentum, M is mass)
% p' = -D & p/M - Kx
% x would be x(1) and p would x(2)

% ODE 45

% Define the system of equations as a function
odeSystem = @(t, x) [x(2) / M; (-D * x(2)) / M - K * x(1)];

% Set initial conditions
initialConditions = [0; 10*M];

% Time span for the simulation
tSpan = [0, 10];

% Solve the ODE
[t, x] = ode45(odeSystem, tSpan, initialConditions);

% Plot functions
figure;
subplot(2, 1, 1);
plot(t, x(:, 1), 'b-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('Position vs. Time');
grid on;

subplot(2, 1, 2);
plot(t, x(:, 2), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Momentum (Ns)');
title('Momentum vs. Time');
grid on;

% Add legends for clarity
subplot(2, 1, 1);
legend('Position');
subplot(2, 1, 2);
legend('Momentum');