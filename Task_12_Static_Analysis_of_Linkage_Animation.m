% Circle Intersection - CoPilot Assist

clc
clear
% Joint coordinates
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 34 0];
F = [43 32 0];
G = [45 17 0];

% Saving original values
originalB=B;
originalC=C;
originalE=E;
originalF=F;

% Length of each link or important links
AB = norm(A-B);
BC = norm(B-C);
CD = norm(C-D);
BE = norm(B-E);

% Initial link lines (handles)
hAB = plot([A(1) originalB(1)], [A(2) originalB(2)], '-b', 'LineWidth', 2);
hBC = plot([originalB(1) originalC(1)], [originalB(2) originalC(2)], '-c', 'LineWidth', 2);
hCD = plot([originalB(1) D(1)], [originalC(2) D(2)], '-k', 'LineWidth', 2);
hBE = plot([originalB(1) originalE(1)], [originalB(2) originalE(2)], '--g', 'LineWidth', 2);
hCE = plot([originalC(1) originalE(1)], [originalC(2) originalE(2)], '--k', 'LineWidth', 2);
hEF = plot([originalE(1) originalF(1)], [originalE(2) originalF(2)], '--m', 'LineWidth', 2);
hFG = plot([originalF(1) G(1)], [originalF(2) G(2)], ':', 'Color', [0.5 0 0.5], 'LineWidth', 2);

% Joint markers
hB = plot(originalB(1), originalB(2), 'bo', 'MarkerFaceColor', 'b');
hC = plot(originalC(1), originalC(2), 'ro', 'MarkerFaceColor', 'r');
hE = plot(originalE(1), originalE(2), 'go', 'MarkerFaceColor', 'g');
hF = plot(originalF(1), originalF(2), 'mo', 'MarkerFaceColor', 'm');

% Trajectory
hTrajB = animatedline('Color', 'b', 'LineStyle', '-', 'LineWidth', 1.5);
hTrajC = animatedline('Color', 'r', 'LineStyle', '-', 'LineWidth', 1.5);
hTrajE = animatedline('Color', 'g', 'LineStyle', '--', 'LineWidth', 1.5);
hTrajF = animatedline('Color', 'm', 'LineStyle', ':', 'LineWidth', 1.5);

legend({'A', 'D', 'G', 'AB', 'BC', 'CD', 'BE', 'CE', 'EF', 'FG', 'B', 'C', 'E', 'F'}, 'Location', 'bestoutside');

% Set axis limits with margin
allx = [A(1), D(1), G(1), originalB(1), originalC(1), originalE(1), originalF(1)];
ally = [A(2), D(2), G(2), originalB(2), originalC(2), originalE(2), originalF(2)];
xmin = min(allx(:)) - 15; xmax = max(allx(:)) + 15;
ymin = min(ally(:)) - 15; ymax = max(ally(:)) + 15;
axis([xmin xmax ymin ymax]);

% Animate
frameDelay = 0.02;
for k = 1:n+1
    % Update Links
    set(hAB, 'XData', [A(1) new_B_joint_x(k)], 'YData', [A(2) new_B_joint_y(k)]);
    set(hBC, 'XData', [new_B_joint_x(k) new_C_joint_x(k)], 'YData', [new_B_joint_y(k) new_C_joint_y(k)]);
    set(hCD, 'XData', [new_C_joint_x(k) D(1)], 'YData', [new_B_joint_y(k) D(2)]);
    set(hBE, 'XData', [new_B_joint_x(k) new_E_joint_x(k)], 'YData', [new_B_joint_y(k) new_E_joint_y(k)]);
    set(hCE, 'XData', [new_C_joint_x(k) new_E_joint_x(k)], 'YData', [new_C_joint_y(k) new_E_joint_y(k)]);
    set(hEF, 'XData', [new_E_joint_x(k) new_F_joint_x(k)], 'YData', [new_E_joint_y(k) new_F_joint_y(k)]);
    set(hFG, 'XData', [new_F_joint_x(k) G(1)], 'YData', [new_F_joint_y(k) G(2)]);

    % Update joint markers
    set(hB, 'XData', new_B_joint_x(k), 'YData', new_B_joint_y(k));
    set(hC, 'XData', new_C_joint_x(k), 'YData', new_C_joint_y(k));
    set(hE, 'XData', new_E_joint_x(k), 'YData', new_E_joint_y(k));
    set(hF, 'XData', new_F_joint_x(k), 'YData', new_F_joint_y(k));

    % Trajectory
    addpoints(hTrajB, new_B_joint_x(k), new_B_joint_y(k));
    addpoints(hTrajC, new_C_joint_x(k), new_C_joint_y(k));
    addpoints(hTrajE, new_E_joint_x(k), new_E_joint_y(k));
    addpoints(hTrajF, new_F_joint_x(k), new_F_joint_y(k));

    drawnow;
    pause(frameDelay);
end