% Static Analysis Linkage MATLAB Code

clc
clear

%define joint coordinates
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 34 0];
F = [43 32 0];
G = [45 17 0];

AB = norm(A-B);
BC = norm(B-C);
CD = norm(C-D);
BE = norm(B-E);
CE = norm(C-E);
EF = norm(E-F);
FG = norm(F-G);

% Compute the initial angle of link AB, input link
initial_angle_AB = atan2(B(2)-A(2),B(1)-A(1));

% If this angle is negative, add / subtract 2 * pi
if (initial_angle_AB < 0)
    initial_angle_AB = initial_angle_AB + (2 * pi);
end

% Storing values of B
numSteps = 360;
new_B_joint_x = zeros(1, numSteps + 1);
new_B_joint_y = zeros(1, numSteps + 1);

new_B_joint_x(1) = B(1);
new_B_joint_y(1) = B(2);

% Storing values of C
new_C_joint_x = zeros(1, numSteps + 1);
new_C_joint_y = zeros(1, numSteps + 1);

new_C_joint_x(1) = C(1);
new_C_joint_y(1) = C(2);


% For-loop
for theta = 1:1:360
    B_new = A + [AB * cos(initial_angle_AB + deg2rad(theta)) AB * sin(initial_angle_AB + deg2rad(theta)) 0];

    new_B_joint_x(theta + 1) = B_new(1);
    new_B_joint_y(theta + 1) = B_new(2);

    % Compute the intersections and obtain new joint C
    % with D as center, CD as radius, need another circle

    % Compute the intersection points of the two circles
    syms x y
    eq1 = (x -B_new(1))^2 + (y - B_new(2))^2 == BC^2;
    eq2 = (x - D(1))^2 + (y - D(2))^2 == CD^2;

    % Solve the system of equations
    intersectionPoints = solve([eq1, eq2], [x, y]);

    % Assumption is that there will be a point generated
    C_new_1 = double([intersectionPoints.x(1), intersectionPoints.y(1), 0]); % Take the first intersection
    C_new_2 = double([intersectionPoints.x(2), intersectionPoints.y(2), 0]);

    % Compare new Cs with the previous C value
    % Compare new C positions with the original C value
    dist_C1 = norm(C_new_1 -C);
    dist_C2 = norm(C_new_2 -C);

    % Whichever distance is the smallest, that C will be the new_C
    if dist_C1 < dist_C2
        C_new = C_new_1;
    else
        C_new = C_new_2;
    end

    new_C_joint_x(theta + 1) = C_new(1);
    new_C_joint_y(theta + 1) = C_new(2);

    % Another method of solving:
    % circcirc function
end

% Plot the trajectory of point B
figure;
plot(new_B_joint_x, new_B_joint_y, 'b-', 'LineWidth', 2);
hold on;
plot(A(1), A(2), 'ro', 'MarkerSize', 8, 'DisplayName', 'Point A');
plot(B(1), B(2), 'go', 'MarkerSize', 8, 'DisplayName', 'Initial Point B');
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Trajectory of Point B');
legend show;
grid on;