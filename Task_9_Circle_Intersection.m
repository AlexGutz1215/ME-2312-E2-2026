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
