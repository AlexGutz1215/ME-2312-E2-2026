%% Problem 1: Add Two User Inputs + Solve Simultaneous Equations
% This script asks the user for two numbers, adds them, and displays
% the result in the format: first+second=result
% It also demonstrates solving two simultaneous equations symbolically.

clear; clc;

% Part A - Add two user-input numbers
num1 = input('Enter the first number: ');
num2 = input('Enter the second number: ');

result = num1 + num2;

% Display in the format: first+second=result  (e.g., 2+3=5)
fprintf('%g+%g=%g\n', num1, num2, result);

% Part B - Solve two simultaneous equations (symbolic method)
% Solving for x and y symbolically using MATLAB's Symbolic Math Toolbox

syms x y

eq1 = x + y == 10;
eq2 = 2*x - y == 2;

solution = solve([eq1, eq2], [x, y]);

fprintf('\nSolving the system of equations:\n');
fprintf('  x + y = 10\n');
fprintf('  2x - y = 2\n\n');

fprintf('x = %g\n', double(solution.x));
fprintf('y = %g\n', double(solution.y));