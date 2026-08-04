% Static analysis of linkage with MATLAB code
clc
clear

% Define coordinates
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

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
eqn10 = cross(F-S4, -FF) + cross(G-S5, FG) == 0;

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

% Print functions'
disp(['Static Torque: ' num2str(StaticTorque)]);