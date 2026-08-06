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

% Storing values of E
new_E_joint_x = zeros(1, numSteps + 1);
new_E_joint_y = zeros(1, numSteps + 1);
new_E_joint_x(1) = E(1);
new_E_joint_y(1) = E(2);

% Storing values of F
new_F_joint_x = zeros(1, numSteps + 1);
new_F_joint_y = zeros(1, numSteps + 1);
new_F_joint_x(1) = F(1);
new_F_joint_y(1) = F(2);

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
    % What if there are no intersection points

    % Check if intersection points exist
    if isempty(intersectionPoints)
        C_new_1 = double([intersectionPoints.x(1), intersectionPoints.y(1), 0]);
        C_new_2 = double([intersectionPoints.x(2), intersectionPoints.y(2), 0]);

        dist_C1 = norm(C_new_1 -C);
        dist_C2 = norm(C_new_2 -C);

        if dist_C1 < dist_C2
            C_new = C_new_1;
        else
            C_new = C_new_2;
        end
        new_C_joint_x(theta + 1) = C_new(1);
        new_C_joint_y(theta + 1) = C_new(2);
    else
        fprintf('New Position cannot be determined at angle: %d degree fron the intial position', theta);
    end

    % Compute the new position of joint E
    % Circle with New B as center and BE as radius and
    % Circle with New C as center and CE as radius
    % Compute the intersection points for joint E
    eq3 = (x - B_new(1))^2 + (y - B_new(2))^2 == BE^2;
    eq4 = (x - C_new(1))^2 + (y - C_new(2))^2 == CE^2;

    % Solve the system of equations for E
    intersectionPointsE = solve([eq3, eq4], [x, y]);

    % Check if intersection points exist for E
    if ~isempty(intersectionPointsE)
        E_new_1 = double([intersectionPointsE.x(1), intersectionPointsE.y(1), 0]);
        E_new_2 = double([intersectionPointsE.x(2), intersectionPointsE.y(2), 0]);

        dist_E1 = norm(E_new_1 - E);
        dist_E2 = norm(E_new_2 - E);

        if dist_E1 < dist_E2
            E_new = E_new_1;
        else
            E_new = E_new_2;
        end

        new_E_joint_x(theta + 1) = E_new(1);
        new_E_joint_y(theta + 1) = E_new(2);
    else
        fprintf('New Position for E cannot be determined at angle: %d degree from the initial position\n', theta);
    end

    % New position of joint F
    % Compute the intersection points for joint F
    eq5 = (x - E_new(1))^2 + (y - E_new(2))^2 == EF^2;
    eq6 = (x - F(1))^2 + (y - F(2))^2 == FG^2;

    % Solve the system of equations for F
    intersectionPointsF = solve([eq5, eq6], [x, y]);

    % Check if intersection points exist for F
    if ~isempty(intersectionPointsF)
        F_new_1 = double([intersectionPointsF.x(1), intersectionPointsF.y(1), 0]);
        F_new_2 = double([intersectionPointsF.x(2), intersectionPointsF.y(2), 0]);

        dist_F1 = norm(F_new_1 - F);
        dist_F2 = norm(F_new_2 - F);

        if dist_F1 < dist_F2
            F_new = F_new_1;
        else
            F_new = F_new_2;
        end

        new_F_joint_x(theta + 1) = F_new(1);
        new_F_joint_y(theta + 1) = F_new(2);
    else
        fprintf('New Position for F cannot be determined at angle: %d degree from the initial position\n', theta);
    end

    % Computed new positions of B, C, E, F
    B = B_new;
    C = C_new;
    E = E_new;
    F = F_new;

end



% Another method of solving:
% circcirc function

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

% Plot the trajectory of joints C, E, and F
plot(new_C_joint_x, new_C_joint_y, 'r-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point C');
plot(new_E_joint_x, new_E_joint_y, 'm-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point E');
plot(new_F_joint_x, new_F_joint_y, 'c-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point F');
legend show;