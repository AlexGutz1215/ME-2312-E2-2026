% Static analysis of six-bar linkage (Image 1 configuration)
% Grounded joints: A, D, G (all revolute joints)
% Motor/input torque applied at A
% Ternary link: D-C-E (grounded at D)
% Artifact H acts on link FG, 1.843 m from F, extended beyond F (away from G)
clc
clear

% Define coordinates
A = [1.4 0.485 0];
B = [1.67 0.99 0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.54 0];
F = [-0.98 2.57 0];
G = [0.05 0.2 0];

% Location of artifact H: on link FG, 1.843 m from F, extended beyond F
% (away from G, matching the mount shown above F in the figure)
Vector_FG = (F - G) / norm(F - G);   % unit vector pointing from G toward F
H = F + 1.843 * Vector_FG;
 
% Define center of mass for each link
S1 = (A + B) / 2;         % COM of link AB (binary)
S2 = (B + C) / 2;         % COM of link BC (binary)
S3 = (D + C + E) / 2;     % COM of link DCE (ternary)
S4 = (E + F) / 2;         % COM of link EF (binary)
S5 = (H + F + G) / 2;     % COM of link FG (binary)
                          % Link FG extends beyond F, so assume total
                          % length goes do H
 
% Static equilibrium equations using symbolic method
syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin
 
% Define force vectors (joint forces)
FA = [FAx FAy 0];   % ground reaction at A (on link AB)
FB = [FBx FBy 0];   % force on link AB from link BC, at joint B
FC = [FCx FCy 0];   % force on link BC from link DCE, at joint C
FD = [FDx FDy 0];   % ground reaction at D (on link DCE)
FE = [FEx FEy 0];   % force on link DCE from link EF, at joint E
FF = [FFx FFy 0];   % force on link EF from link FG, at joint F
FG = [FGx FGy 0];   % ground reaction at G (on link FG)
Torque = [0 0 Tin];  % input torque applied at A
 
% Mass of each link (kg) -- ASSUMPTION: not specified, using 10 kg to
% match the sample. Update if actual link masses are given.
Mass_AB  = 10;
Mass_BC  = 10;
Mass_DCE = 10;
Mass_EF  = 10;
Mass_FG  = 10;
 
% Artifact load acting at H (downward, due to gravity)
% PLACEHOLDER: actual artifact mass was not provided
Mass_Artifact = 5;   % kg (PLACEHOLDER -- UPDATE THIS)
Artifact_Input = [0, -Mass_Artifact*9.81, 0];
 
% Equilibrium equations
 
% Link AB (grounded at A, motor input)
% Sum of Forces = 0: FA + FB + W_AB = 0
Weight_AB = [0 -Mass_AB*9.81 0];
eqn1 = FA + FB + Weight_AB == 0;
% Sum of Torques about S1 = 0: S1A x FA + S1B x FB + Tin = 0
eqn2 = cross(A-S1, FA) + cross(B-S1, FB) + Torque == 0;
 
% Link BC (binary)
% Sum of Forces = 0: -FB + FC + W_BC = 0
Weight_BC = [0 -Mass_BC*9.81 0];
eqn3 = -FB + FC + Weight_BC == 0;
% Sum of Torques about S2 = 0: S2B x -FB + S2C x FC = 0
eqn4 = cross(B-S2, -FB) + cross(C-S2, FC) == 0;
 
% Link DCE (ternary, grounded at D)
% Sum of Forces = 0: FD - FC + FE + W_DCE = 0
Weight_DCE = [0 -Mass_DCE*9.81 0];
eqn5 = FD - FC + FE + Weight_DCE == 0;
% Sum of Torques about S3 = 0: S3D x FD + S3C x -FC + S3E x FE = 0
eqn6 = cross(D-S3, FD) + cross(C-S3, -FC) + cross(E-S3, FE) == 0;
 
% Link EF (binary)
% Sum of Forces = 0: -FE + FF + W_EF = 0
Weight_EF = [0 -Mass_EF*9.81 0];
eqn7 = -FE + FF + Weight_EF == 0;
% Sum of Torques about S4 = 0: S4E x -FE + S4F x FF = 0
eqn8 = cross(E-S4, -FE) + cross(F-S4, FF) == 0;
 
% Link FG (binary, grounded at G, carries artifact load at H)
% Sum of Forces = 0: -FF + FG + W_FG + Artifact_Input = 0
Weight_FG = [0 -Mass_FG*9.81 0];
eqn9 = -FF + FG + Weight_FG + Artifact_Input == 0;
% Sum of Torques about S5 = 0:
% S5F x -FF + S5G x FG + S5H x Artifact_Input = 0
eqn10 = cross(F-S5, -FF) + cross(G-S5, FG) + cross(H-S5, Artifact_Input) == 0;
 
% Solving Equations
eqns = [eqn1; eqn2; eqn3; eqn4; eqn5; eqn6; eqn7; eqn8; eqn9; eqn10];
solution = solve(eqns, [FAx, FAy, FBx, FBy, FCx, FCy, FDx, FDy, FEx, FEy, FFx, FFy, FGx, FGy, Tin]);
 
% Extracting the numerical values from the solution
FA = double([solution.FAx, solution.FAy, 0]);
FB = double([solution.FBx, solution.FBy, 0]);
FC = double([solution.FCx, solution.FCy, 0]);
FD = double([solution.FDx, solution.FDy, 0]);
FE = double([solution.FEx, solution.FEy, 0]);
FF = double([solution.FFx, solution.FFy, 0]);
FG = double([solution.FGx, solution.FGy, 0]);
StaticTorque = double(solution.Tin);
 
% Print results
disp(['Static Torque at A: ' num2str(StaticTorque) ' N*m']);
disp('Reaction/joint forces (N):');
disp(['FA = ' num2str(FA(1)) ', ' num2str(FA(2))]);
disp(['FB = ' num2str(FB(1)) ', ' num2str(FB(2))]);
disp(['FC = ' num2str(FC(1)) ', ' num2str(FC(2))]);
disp(['FD = ' num2str(FD(1)) ', ' num2str(FD(2))]);
disp(['FE = ' num2str(FE(1)) ', ' num2str(FE(2))]);
disp(['FF = ' num2str(FF(1)) ', ' num2str(FF(2))]);
disp(['FG = ' num2str(FG(1)) ', ' num2str(FG(2))]);
