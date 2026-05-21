%% ─────────────────────────────────────────────────────────────
% File        : draw_robot_configs.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Shows the ABB IRB 1200 in two reference configurations:
%               (a) Home position — all joints at 0 rad
%               (b) q1 = +90° — base rotated 90° about Z
%               Saved as results/figures/robot_configs.png (Fig 4.2).
% Inputs      : robot — rigidBodyTree (DataFormat='column')
% Outputs     : none (figure saved as side-effect)
% Dependencies: show (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function draw_robot_configs(robot)

narginchk(1, 1);

q_home = zeros(6, 1);                      % Home: all joints at 0°
q_q1p  = deg2rad([90; 0; 0; 0; 0; 0]);    % q1 = +90°, rest at 0°

fig = figure('Name', 'IRB 1200 — Reference Configurations', ...
             'NumberTitle', 'off', 'Position', [100, 100, 1100, 520]);

%% ── Left panel: Home position ─────────────────────────────────
ax1 = subplot(1, 2, 1);
show(robot, q_home, 'Frames', 'off', 'PreservePlot', false, 'Parent', ax1);
patches = findobj(ax1, 'Type', 'Patch');
set(patches, 'FaceColor', [0.55, 0.55, 0.60], 'FaceAlpha', 0.90);
hold(ax1, 'on');
% Mark EE with a star
T_ee = getTransform(robot, q_home, robot.BodyNames{end});
plot3(ax1, T_ee(1,4), T_ee(2,4), T_ee(3,4), 'r*', ...
      'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'End-effector');
hold(ax1, 'off');
axis(ax1, 'equal');  grid(ax1, 'on');
xlabel(ax1, 'X [m]');  ylabel(ax1, 'Y [m]');  zlabel(ax1, 'Z [m]');
view(ax1, 35, 25);
title(ax1, {'Home position', 'q = [0°, 0°, 0°, 0°, 0°, 0°]'}, 'FontSize', 11);
xlim(ax1, [-0.60, 0.60]);  ylim(ax1, [-0.60, 0.60]);  zlim(ax1, [-0.05, 1.05]);

%% ── Right panel: q1 = +90° ────────────────────────────────────
ax2 = subplot(1, 2, 2);
show(robot, q_q1p, 'Frames', 'off', 'PreservePlot', false, 'Parent', ax2);
patches2 = findobj(ax2, 'Type', 'Patch');
set(patches2, 'FaceColor', [0.20, 0.50, 0.80], 'FaceAlpha', 0.90);
hold(ax2, 'on');
T_ee2 = getTransform(robot, q_q1p, robot.BodyNames{end});
plot3(ax2, T_ee2(1,4), T_ee2(2,4), T_ee2(3,4), 'r*', ...
      'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'End-effector');
hold(ax2, 'off');
axis(ax2, 'equal');  grid(ax2, 'on');
xlabel(ax2, 'X [m]');  ylabel(ax2, 'Y [m]');  zlabel(ax2, 'Z [m]');
view(ax2, 35, 25);
title(ax2, {'q_1 = +90° (base rotation)', 'q = [90°, 0°, 0°, 0°, 0°, 0°]'}, ...
      'FontSize', 11);
xlim(ax2, [-0.60, 0.60]);  ylim(ax2, [-0.60, 0.60]);  zlim(ax2, [-0.05, 1.05]);

sgtitle('ABB IRB 1200 — Reference Joint Configurations', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'robot_configs');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_robot_configs] Figure saved: %s.png\n', out_base);

end
