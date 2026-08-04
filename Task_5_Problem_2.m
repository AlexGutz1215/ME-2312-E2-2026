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