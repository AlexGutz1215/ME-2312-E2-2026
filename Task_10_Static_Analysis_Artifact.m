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
FH = 1.843; % Distance from F to point H

% Define initial payload value at H (Artifact force)
Payload = 0;

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

% Initial Position of H
FG_vector = G - F;
FG_unit = FG_vector / norm(FG_vector);

% H lies on the extension of FG beyond F
H = F - FH*FG_unit;

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

new_H_joint_x = zeros(1,numSteps+1);
new_H_joint_y = zeros(1,numSteps+1);
new_H_joint_x(1) = H(1);
new_H_joint_y(1) = H(2);

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

    % --- Joint E: Determine E by rotating rigid body CDE ---
    vDC = C_new-D;
    currentRigidAngle = atan2(vDC(2),vDC(1));
    deltaTheta = currentRigidAngle-initialRigidAngle;

    R = [cos(deltaTheta) -sin(deltaTheta);
         sin(deltaTheta)  cos(deltaTheta)];

    rotatedVector = R*vDE0(1:2)';

    E_new = [D(1)+rotatedVector(1), D(2)+rotatedVector(2), 0];

    new_E_joint_x(theta+1)=E_new(1);
    new_E_joint_y(theta+1)=E_new(2);

    % --- Joint F: intersection of circle(E_new, EF) and circle(G, FG) ---
    [F_new, successF] = getNextJointPosition(E_new, EF, G, FG, F);
    if ~successF
        reportFailure('F', theta);
        break;
    end
    new_F_joint_x(theta + 1) = F_new(1);
    new_F_joint_y(theta + 1) = F_new(2);

    % Determine Point H
    FG_vector = G - F_new;
    FG_unit = FG_vector / norm(FG_vector);

    % H is rigidly attached to the FGH link
    H_new = F_new - FH*FG_unit;

    new_H_joint_x(theta+1) = H_new(1);
    new_H_joint_y(theta+1) = H_new(2);

    % Update joint positions for next iteration
    B = B_new;
    C = C_new;
    E = E_new;
    F = F_new;

    % Static analysis
    forcesAndTorque = staticAnalysis(A, B, C, D, E, F, G, H, Payload);

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

plot(new_B_joint_x, new_B_joint_y, 'b-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point B');
hold on;
plot(new_C_joint_x, new_C_joint_y, 'r-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point C');
plot(new_E_joint_x, new_E_joint_y, 'm-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point E');
plot(new_F_joint_x, new_F_joint_y, 'c-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point F');

plot(new_H_joint_x, new_H_joint_y, 'k-', 'LineWidth', 2, 'DisplayName', 'Trajectory of Point H')

plot(A(1), A(2), 'ko','MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Ground pivot A');
plot(D(1), D(2), 'k^','MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Ground pivot D');
plot(G(1), G(2), 'ks','MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Ground pivot G');

plot(new_B_joint_x(1), new_B_joint_y(2), 'go', 'MarkerSize', 8, 'DisplayName', 'Original Point B');

xlabel('X (m)');
ylabel('Y (m)');

title('Six-Bar Linkage Joint Trajectories');

legend('B','C','E','F', 'H', 'A', 'D', 'G');

grid on;
axis equal;

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

function forces = staticAnalysis(A, B, C, D, E, F, G, H, Payload)

% Center of Mass
S1 = (A + B)/2;
S2 = (C + D + E)/3;
S3 = (E + F)/2;
S4 = (F + G)/2;
S5 = (F + G) / 2;

% Static equilibrium equations using symbolic method
syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

% Define force vectors
FA = [FAx FAy 0];
FB = [FBx FBy 0];
FC = [FCx FCy 0];
FD = [FDx FDy 0];
FE = [FEx FEy 0];
FF = [FFx FFy 0];
FG = [FGx FGy 0];
Torque = [0 0 Tin];

% Mass of Each Link
Mass_AB  = 10;
Mass_BC  = 10;
Mass_CDE = 10;
Mass_EF  = 10;
Mass_FG  = 10;

% Applied load
Force_Input = [50 0 0];

% Equilibrium equations
% Link AB
% Sum of Forces = 0; Fa + Fb + W_Ab = 0
Weight_AB = [0 -Mass_AB*9.81 0];
eqn1 = FA + FB + Weight_AB == 0;
% Sum of Torques = 0; S1A x Fa + S1B x Fb + Tin = 0;
eqn2 = cross(A-S1, FA) + cross(B-S1, FB) + Torque == 0;

% Link BC
Weight_BC = [0 -Mass_BC*9.81 0];
eqn3 = -FB + FC + Weight_BC == 0;
eqn4 = cross(B-S2,-FB) + cross(C-S2,FC) == 0;

% Link CDE
Weight_CDE = [0 -Mass_CDE*9.81 0];
eqn5 = -FC + FD + FE + Weight_CDE == 0;
eqn6 = cross(C-S3,-FC) + cross(D-S3,FD) + cross(E-S3,FE) == 0;

% Link EF
Weight_EF = [0 -Mass_EF*9.81 0];
eqn7 = -FE + FF + Weight_EF == 0;
eqn8 = cross(E-S4,-FE) + cross(F-S4,FF) == 0;

% Link FG
Weight_FG = [0 -Mass_FG*9.81 0];
Force_H = [0 -Payload 0];
eqn9 = -FF + FG + Force_H + Weight_FG == 0;
eqn10 = cross(F-S5,-FF) + cross(G-S5,FG) + cross(H-S5,Force_H) == 0;

% Solving Equations
eqns = [eqn1; eqn2; eqn3; eqn4; eqn5; eqn6; eqn7; eqn8; eqn9; eqn10];
solution = solve(eqns, [FAx, FAy, FBx, FBy, FCx, FCy, FDx, FDy, FEx, FEy, FFx, FFy, FGx, FGy, Tin]);

% Extracting the numerical values from the solution
FA = double([solution.FAx, solution.FAy, 0]);
FB = [solution.FBx, solution.FBy, 0];
FC = [solution.FCx, solution.FCy, 0];
FD = [solution.FDx, solution.FDy, 0];
FE = [solution.FEx, solution.FEy, 0];
FF = [solution.FFx, solution.FFy, 0];
FG = [solution.FGx, solution.FGy, 0];
StaticTorque = double(solution.Tin);

forces = [FA; FB; FC; FD; FE; FF; FG; [0 0 StaticTorque]];

end