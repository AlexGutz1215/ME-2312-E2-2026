% Static Analysis Linkage MATLAB Code
% Refactored to use functions for repeated circle-intersection logic
% Linkage:
% Link 1 - AB (Input Crank)
% Link 2 - BC
% Link 3 - CDE (Rigid Ternary Link)
% Link 4 - EF
% Link 5 - FG
%
% Grounded Joints:
% A, D, G

clc
clear

% Define joint coordinates
A = [1.400 0.485 0];
B = [1.670 0.990 0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.540 0];
F = [-0.980 2.570 0];
G = [0.050 0.200 0];

AB = norm(A-B);
BC = norm(B-C);
CD = norm(C-D);
DE = norm(D-E);
EF = norm(E-F);
FG = norm(F-G);

% Compute the initial angle of link AB, input link
initial_angle_AB = atan2(B(2)-A(2), B(1)-A(1));

% If this angle is negative, add 2*pi
if (initial_angle_AB < 0)
    initial_angle_AB = initial_angle_AB + (2 * pi);
end

% Initial Orientation of Rigid Link CDE
vDC0 = C-D;
vDE0 = E-D;
initialRigidAngle = atan2(vDC0(2),vDC0(1));

% Preallocate storage for joint trajectories
numSteps = 360;
new_B_joint_x = zeros(1, numSteps + 1);
new_B_joint_y = zeros(1, numSteps + 1);
new_B_joint_x(1) = B(1);
new_B_joint_y(1) = B(2);

new_C_joint_x = zeros(1, numSteps + 1);
new_C_joint_y = zeros(1, numSteps + 1);
new_C_joint_x(1) = C(1);
new_C_joint_y(1) = C(2);

new_E_joint_x = zeros(1, numSteps + 1);
new_E_joint_y = zeros(1, numSteps + 1);
new_E_joint_x(1) = E(1);
new_E_joint_y(1) = E(2);

new_F_joint_x = zeros(1, numSteps + 1);
new_F_joint_y = zeros(1, numSteps + 1);
new_F_joint_x(1) = F(1);
new_F_joint_y(1) = F(2);

% Main loop
for theta = 1:1:360

    theta

    B_new = A + [AB * cos(initial_angle_AB + deg2rad(theta)), ...
                 AB * sin(initial_angle_AB + deg2rad(theta)), 0];

    new_B_joint_x(theta + 1) = B_new(1);
    new_B_joint_y(theta + 1) = B_new(2);

    % --- Joint C: intersection of circle(B_new, BC) and circle(D, CD) ---
    [C_new, successC] = getNextJointPosition(B_new, BC, D, CD, C);
    if ~successC
        reportFailure('C', theta);
        break;
    end
    new_C_joint_x(theta + 1) = C_new(1);
    new_C_joint_y(theta + 1) = C_new(2);

    % --- Joint E: intersection of circle(B_new, BE) and circle(C_new, CE) ---
    [E_new, successE] = getNextJointPosition(B_new, BE, C_new, CE, E);
    if ~successE
        reportFailure('E', theta);
        break;
    end
    new_E_joint_x(theta + 1) = E_new(1);
    new_E_joint_y(theta + 1) = E_new(2);

    % --- Joint F: intersection of circle(E_new, EF) and circle(G, FG) ---
    [F_new, successF] = getNextJointPosition(E_new, EF, G, FG, F);
    if ~successF
        reportFailure('F', theta);
        break;
    end
    new_F_joint_x(theta + 1) = F_new(1);
    new_F_joint_y(theta + 1) = F_new(2);

    % Update joint positions for next iteration
    B = B_new;
    C = C_new;
    E = E_new;
    F = F_new;

end

% After loop: trim pre-allocated arrays to actual lengths
lastIdx = theta;

new_B_joint_x = new_B_joint_x(1:lastIdx);
new_B_joint_y = new_B_joint_y(1:lastIdx);
new_C_joint_x = new_C_joint_x(1:lastIdx);
new_C_joint_y = new_C_joint_y(1:lastIdx);
new_E_joint_x = new_E_joint_x(1:lastIdx);
new_E_joint_y = new_E_joint_y(1:lastIdx);
new_F_joint_x = new_F_joint_x(1:lastIdx);
new_F_joint_y = new_F_joint_y(1:lastIdx);

% Plot the trajectories
figure;
plot(new_B_joint_x, new_B_joint_y, 'b-', 'LineWidth', 2);
hold on;
plot(A(1), A(2), 'ro', 'MarkerSize', 8, 'DisplayName', 'Point A');
plot(new_B_joint_x(1), new_B_joint_y(2), 'go', 'MarkerSize', 8, 'DisplayName', 'Original Point B');
xlabel('X Coordinate');
ylabel('Y Coordinate');
title('Trajectory of all Joints');
legend show;
grid on;

plot(new_C_joint_x, new_C_joint_y, 'r-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point C');
plot(new_E_joint_x, new_E_joint_y, 'm-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point E');
plot(new_F_joint_x, new_F_joint_y, 'c-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point F');
legend show;
grid on;

% ======================= Local Functions =======================

function [newPoint, success] = getNextJointPosition(center1, r1, center2, r2, refPoint)
% GETNEXTJOINTPOSITION Finds the new position of a joint as the
% intersection of two circles, choosing the intersection point closest
% to the joint's previous position.
%
%   center1, center2 : [x y z] centers of the two circles
%   r1, r2            : radii of the two circles (link lengths)
%   refPoint          : previous position of the joint, used to pick
%                       between the two intersection candidates
%
%   newPoint : the selected [x y z] intersection point (NaNs if none)
%   success  : true if a valid real intersection was found
    [P1, P2, isValid] = circleCircleIntersection(center1, r1, center2, r2);

    if ~isValid
        newPoint = [NaN NaN NaN];
        success = false;
        return;
    end

    newPoint = selectNearestPoint(P1, P2, refPoint);
    success = true;
end


function [P1, P2, isValid] = circleCircleIntersection(c1, r1, c2, r2)
% CIRCLECIRCLEINTERSECTION Computes the (up to two) intersection points
% of two circles defined by center/radius pairs, using symbolic solve.
%
%   c1, c2 : [x y z] circle centers
%   r1, r2 : circle radii
%
%   P1, P2  : the two intersection points (NaNs if not valid)
%   isValid : true only if two fully real solutions were found
    syms x y

    eq1 = (x - c1(1))^2 + (y - c1(2))^2 == r1^2;
    eq2 = (x - c2(1))^2 + (y - c2(2))^2 == r2^2;

    sol = solve([eq1, eq2], [x, y]);
    sx = sol.x;
    sy = sol.y;

    % Keep only fully real solutions
    isRealX = arrayfun(@(e) isAlways(imag(e) == 0), sx);
    isRealY = arrayfun(@(e) isAlways(imag(e) == 0), sy);
    realIdx = find(isRealX & isRealY);

    if numel(realIdx) < 2
        P1 = [NaN NaN 0];
        P2 = [NaN NaN 0];
        isValid = false;
        return;
    end

    P1 = double([sx(realIdx(1)), sy(realIdx(1)), 0]);
    P2 = double([sx(realIdx(2)), sy(realIdx(2)), 0]);
    isValid = true;
end


function point = selectNearestPoint(P1, P2, refPoint)
% SELECTNEARESTPOINT Returns whichever of P1, P2 is closer to refPoint.
% Used to keep the linkage moving continuously rather than flipping
% to the other assembly branch between steps.
    dist1 = norm(P1 - refPoint);
    dist2 = norm(P2 - refPoint);

    if dist1 < dist2
        point = P1;
    else
        point = P2;
    end
end

function reportFailure(jointName, theta)
% REPORTFAILURE Prints a standard message when a joint position cannot
% be determined at a given crank angle.
    fprintf('New Position for %s cannot be determined at angle: %d degree from the initial position\n', ...
        jointName, theta);
end