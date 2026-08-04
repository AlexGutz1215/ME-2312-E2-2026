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