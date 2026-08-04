% Static analysis of linkage with MATLAB code

clr
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