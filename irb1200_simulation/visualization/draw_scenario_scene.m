%% ─────────────────────────────────────────────────────────────
% File        : draw_scenario_scene.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Renders a representative 3D scene for Scenario 2 (central
%               box) showing: robot at start config, obstacles, an operator
%               sphere at a fixed illustrative position, and ISO/TS 15066
%               SSM safety zone wireframes. Saved as
%               results/figures/scenario2_scene.png (Fig 4.3).
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               obstacles — cell array of obstacle structs (Scenario 2)
% Outputs     : none (figure saved as side-effect)
% Dependencies: draw_scene
%% ─────────────────────────────────────────────────────────────

function draw_scenario_scene(robot, obstacles)

narginchk(2, 2);

% Robot in start configuration used for Scenario 2 benchmark
q_start  = deg2rad([-90; 70; 0; 0; 0; 0]);
% Illustrative operator position: approaching from X at 0.85 m (SLOW zone)
op_pos   = [0.85, 0.0, 0.7];
ssm_state = 'SLOW';

fig = figure('Name', 'Scenario 2 — 3D Scene Overview', ...
             'NumberTitle', 'off', 'Position', [100, 100, 900, 700]);

draw_scene(robot, q_start, obstacles, op_pos, ssm_state);

% Additional annotation: start EE marker
T_ee = getTransform(robot, q_start, robot.BodyNames{end});
hold on;
plot3(T_ee(1,4), T_ee(2,4), T_ee(3,4), 'gs', ...
      'MarkerSize', 12, 'MarkerFaceColor', [0.10, 0.75, 0.20], ...
      'LineWidth', 2, 'DisplayName', 'EE (start)');

% Camera for a clear view of the scene
campos([2.2, -2.0, 1.8]);
camtarget([0.30, 0.0, 0.40]);
camup([0, 0, 1]);
camva(22);

legend('Location', 'northeast', 'FontSize', 9);
title({'Scenario 2 — Central Box Obstacle', ...
       'Robot: start config | Operator: SLOW zone (0.85 m) | ISO/TS 15066 SSM'}, ...
      'FontSize', 11);

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'scenario2_scene');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_scenario_scene] Figure saved: %s.png\n', out_base);

end
