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

%% Problem 2: Odd/Even Operation on Two User Inputs
% This script asks the user for two numbers;
%   - adds them if both are odd
%   - subtracts the smaller from the larger if both are even
%   - multiplies them if one is odd and the other is even
% The result is displayed in the command window.
 
clear; clc;
 
num1 = input('Enter the first number: ');
num2 = input('Enter the second number: ');
 
result = oddEvenOperation(num1, num2);
 
fprintf('Result of operation on %g and %g is: %g\n', num1, num2, result);
 
% Local function definition
function result = oddEvenOperation(a, b)
    % oddEvenOperation determines an operation based on the parity of a and b
    %   - Both odd  -> a + b
    %   - Both even -> larger - smaller
    %   - Mixed     -> a * b
 
    aIsOdd = mod(a, 2) ~= 0;
    bIsOdd = mod(b, 2) ~= 0;
 
    if aIsOdd && bIsOdd
        result = a + b;
        fprintf('Both numbers are odd. Adding them together.\n');
    elseif ~aIsOdd && ~bIsOdd
        result = max(a, b) - min(a, b);
        fprintf('Both numbers are even. Subtracting smaller from larger.\n');
    else
        result = a * b;
        fprintf('One number is odd and the other is even. Multiplying them.\n');
    end
end

%% Problem 3: Generate 10 Random Numbers and Plot them
% This script uses a for-loop to generate 10 random numbers and plots
% them with the x-axis representing the index (1 to 10) and the y-axis
% representing the random number generated at that index.

clear; clc;

n = 10;
randomNumbers = zeros(1, n);   % preallocate

for i = 1:n
    randomNumbers(i) = rand() * 100;   % random number between 0 and 100
end

% Display the generated numbers
disp('Generated random numbers:');
disp(randomNumbers);

% Plot the results
figure;
plot(1:n, randomNumbers, '-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'b');
xlabel('Index (1 to 10)');
ylabel('Random Number Value');
title('Plot of 10 Randomly Generated Numbers');
grid on;
