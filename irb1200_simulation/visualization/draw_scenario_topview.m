%% ─────────────────────────────────────────────────────────────
% File        : draw_scenario_topview.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Top-down (zenith, view(0,90)) 2×2 overview of all four
%               test scenarios (Fig 5.1). For each scenario shows the XY
%               obstacle footprints, the EE start and goal positions and
%               the robot base origin. Saved as
%               results/figures/scenario_topview.png (Fig 5.1).
% Inputs      : robot — rigidBodyTree (DataFormat='column')
% Outputs     : none (figure saved as side-effect)
% Dependencies: build_scenario, getTransform
%% ─────────────────────────────────────────────────────────────

function draw_scenario_topview(robot)

narginchk(1, 1);

% Standard benchmark start/goal configurations
q_start = deg2rad([-90; 70; 0; 0; 0; 0]);
q_goal  = deg2rad([ 90; 70; 0; 0; 0; 0]);

T_s = getTransform(robot, q_start, robot.BodyNames{end});
T_g = getTransform(robot, q_goal,  robot.BodyNames{end});
ee_s = T_s(1:3,4)';
ee_g = T_g(1:3,4)';

sc_titles = {'Scenario 1 — Free Space', ...
             'Scenario 2 — Central Box', ...
             'Scenario 3 — Narrow Corridor', ...
             'Scenario 4 — Dense (5 obstacles)'};

fig = figure('Name', 'Scenario Overview — Top View', 'NumberTitle', 'off', ...
             'Position', [50, 50, 1100, 900]);

for sc = 1:4
    obstacles = build_scenario(sc);

    ax = subplot(2, 2, sc);
    hold(ax, 'on');

    % Robot base
    plot(ax, 0, 0, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', [0.3,0.3,0.3], ...
         'DisplayName', 'Robot base');

    % Obstacle footprints (XY projection)
    for k = 1:numel(obstacles)
        obs = obstacles{k};
        T   = obs.T_world;
        clr = obs.color;
        cx  = T(1,4);  cy = T(2,4);

        if isa(obs.geometry, 'collisionBox')
            hx = obs.geometry.X / 2;
            hy = obs.geometry.Y / 2;
            % Corners in local frame
            corners_loc = [hx,hy,0; hx,-hy,0; -hx,-hy,0; -hx,hy,0; hx,hy,0];
            % Rotate by obstacle yaw only (top-down view: use rotation about Z)
            corners_w = (T(1:3,1:3) * corners_loc' + repmat(T(1:3,4),1,5))';
            fill(ax, corners_w(:,1), corners_w(:,2), clr, ...
                 'FaceAlpha', 0.55, 'EdgeColor', clr*0.7, 'LineWidth', 1.5, ...
                 'HandleVisibility', ternary(k==1,'on','off'), ...
                 'DisplayName', 'Obstacle');

        elseif isa(obs.geometry, 'collisionCylinder')
            R = obs.geometry.Radius;
            theta = linspace(0, 2*pi, 50);
            fill(ax, cx + R*cos(theta), cy + R*sin(theta), clr, ...
                 'FaceAlpha', 0.55, 'EdgeColor', clr*0.7, 'LineWidth', 1.5, ...
                 'HandleVisibility', ternary(k==1,'on','off'), ...
                 'DisplayName', 'Obstacle');

        elseif isa(obs.geometry, 'collisionSphere')
            R = obs.geometry.Radius;
            theta = linspace(0, 2*pi, 50);
            fill(ax, cx + R*cos(theta), cy + R*sin(theta), clr, ...
                 'FaceAlpha', 0.50, 'EdgeColor', clr*0.7, 'LineWidth', 1.5, ...
                 'HandleVisibility', 'off');
        end
    end

    % EE start and goal markers
    plot(ax, ee_s(1), ee_s(2), 'go', 'MarkerSize', 11, 'MarkerFaceColor', ...
         [0.1,0.75,0.2], 'LineWidth', 2, 'DisplayName', 'EE start');
    plot(ax, ee_g(1), ee_g(2), 'r*', 'MarkerSize', 11, 'LineWidth', 2, ...
         'DisplayName', 'EE goal');

    % Dashed line between start and goal (direct path)
    plot(ax, [ee_s(1), ee_g(1)], [ee_s(2), ee_g(2)], 'k:', ...
         'LineWidth', 1.0, 'HandleVisibility', 'off');

    axis(ax, 'equal');  grid(ax, 'on');
    xlabel(ax, 'X [m]', 'FontSize', 10);
    ylabel(ax, 'Y [m]', 'FontSize', 10);
    xlim(ax, [-0.15, 0.75]);  ylim(ax, [-0.50, 0.50]);
    view(ax, 0, 90);   % top-down zenith view
    title(ax, sc_titles{sc}, 'FontSize', 11, 'FontWeight', 'bold');
    legend(ax, 'Location', 'northeast', 'FontSize', 8);
    hold(ax, 'off');
end

sgtitle('Test Scenarios — Top-Down View of Obstacle Layout (XY Plane)', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'scenario_topview');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_scenario_topview] Figure saved: %s.png\n', out_base);

end

%% ── Local helper ─────────────────────────────────────────────
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
