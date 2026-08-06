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

% For-loop
for theta = 1:1:360
    B_new = A + [AB * cos(initial_angle_AB + deg2rad(theta)) AB * sin(initial_angle_AB + deg2rad(theta)) 0];
end