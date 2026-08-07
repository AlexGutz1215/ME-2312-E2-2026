% Static Analysis Linkage MATLAB Code
% Refactored to use functions for repeated circle-intersection logic
 
clc
clear
 
% Define joint coordinates
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
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
initial_angle_AB = atan2(B(2)-A(2), B(1)-A(1));
 
% If this angle is negative, add 2*pi
if (initial_angle_AB < 0)
    initial_angle_AB = initial_angle_AB + (2 * pi);
end
 
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

    % Static Analysis for the newly computed position of six-bar linkage
    forcesAndTorque = staticAnalysis(A, B, C, D, E, F, G);
 
end
 
% Another method of solving:
% circcirc function
 
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

rmseVal = comparing_PMKS_MATLAB(new_C_joint_x, new_C_joint_y, D(1), D(2));
 
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

function forces = staticAnalysis(A, B, C, D, E, F, G)
% Forces are not passes and so are masses
% Define center of mass
S1 = (A+B)/2; % COM of link AB
S2 = (B+C+E)/2; % COM of link BCE
S3 = (C+D)/2; % COM of link CD
S4 = (E+F)/2; % COM of link EF
S5 = (F+G)/2; % COM of link FG

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

% Mass of each link
Mass_AB = 10;
Mass_BEC = 10;
Mass_CD = 10;
Mass_EF = 10;
Mass_FG = 10;

% Applied load
Force_Input = [50 0 0];

% Equilibrium equations

% Link AB
% Sum of Forces = 0; Fa + Fb + W_Ab = 0
Weight_AB = [0 -Mass_AB*9.81 0];
eqn1 = FA + FB + Weight_AB == 0;
% Sum of Torques = 0; S1A x Fa + S1B x Fb + Tin = 0;
eqn2 = cross(A-S1, FA) + cross(B-S1, FB) + Torque == 0;

% Link BEC
% Sum of Forces = o; -Fb + Fc + Fe + W_BEC = 0
Weight_BEC = [0 -Mass_BEC*9.81 0];
eqn3 = -FB + FC + FE + Weight_BEC == 0;
% Sum of Torque = 0; S2B x -Fb + S2E x Fe + S2C x Fc = 0;
eqn4 = cross(B-S2, FB) + cross(E-S2, FE) + cross(C-S2, FC) == 0;

% Link CD
% Sum of Forces = 0; -Fc + FD + Weight_CD = 0;
Weight_CD = [0 -Mass_CD*9.81 0];
eqn5 = -FC + FD + Weight_CD == 0;
% Sum of Torques = 0; S3C x -Fc + S3D x Fd = 0;
eqn6 = cross(C-S3, -FC) + cross(D-S3, FD) == 0;

% Link EF
% Sum of Forces = 0; -Fe + Ff + Weight_EF = 0;
Weight_EF = [0 -Mass_EF*9.81 0];
eqn7 = -FE + FF + Weight_EF == 0;
% Sum of Torques = 0;
eqn8 = cross(E-S4, -FE) + cross(F-S4, FF) == 0;

% Link FG
Weight_FG = [0 -Mass_FG*9.81 0];
eqn9 = -FF + FG + Force_Input + Weight_FG ==0;
eqn10 = cross(F-S5, -FF) + cross(G-S5, FG) == 0;

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

function rmse = comparing_PMKS_MATLAB(cx, cy, dx, dy)


%% Set up the Import Options and import the data
opts = spreadsheetImportOptions("NumVariables", 15);

% Specify sheet and range
opts.Sheet = "Sheet1";
opts.DataRange = "A2:O646";

% Specify column names and types
opts.VariableNames = ["Angle", "JointAXCm", "JointAYCm", "JointBXCm", "JointBYCm", "JointCXCm", "JointCYCm", "JointDXCm", "JointDYCm", "JointEXCm", "JointEYCm", "JointFXCm", "JointFYCm", "JointGXCm", "JointGYCm"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Import the data
kinematics_loops7Joints6Links06_Aug20_20_57 = readtable("C:\Users\adgut\Downloads\kinematics_loops7Joints6Links06-Aug 20_20_57.xlsx", opts, "UseExcel", false);


%% Clear temporary variables
clear opts

% Storing values into arrays
pmks_x1 = kinematics_loops7Joints6Links06_Aug20_20_57.JointCXCm;
pmks_y1 = kinematics_loops7Joints6Links06_Aug20_20_57.JointCYCm;

pmks_x2 = kinematics_loops7Joints6Links06_Aug20_20_57.JointDXCm;
pmks_y2 = kinematics_loops7Joints6Links06_Aug20_20_57.JointDYCm;

% Calculating distances for PMKS+ data and then for MATLAB data
distance_PMKS = zeros(1, 42);
distance_MATLAB = zeros(1, 42);

for theta = 1:1:42
    distance_PMKS(theta) = norm([pmks_x1(theta) pmks_y1(theta) 0] - [pmks_x2(theta) pmks_y2(theta) 0]);
    distance_MATLAB(theta) = norm([cx(theta) cy(theta) 0] - [dx dy 0]);
end

rsmeVal = rsme(distance_PMKS, distance_MATLAB);

% Compute RMSE between the two datasets
rmse = sqrt(mean((dist_PMKS - dist_MATLAB).^2));

end