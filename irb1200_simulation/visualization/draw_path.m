%% ─────────────────────────────────────────────────────────────
% File        : draw_path.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Plots the joint-space trajectory as six subplot panels
%               (one per joint), including joint limit lines.
% Inputs      : path           — N×6 double, joint waypoints [rad]
%               algorithm_name — char, label for the figure title
% Outputs     : none (figure created as side-effect)
% Dependencies: get_joint_limits
%% ─────────────────────────────────────────────────────────────

function draw_path(path, algorithm_name)

narginchk(1, 2);
if nargin < 2 || isempty(algorithm_name), algorithm_name = 'Planner'; end

assert(size(path, 2) == 6, ...
    'draw_path: path must be N×6. Got N×%d.', size(path, 2));

limits = get_joint_limits();   % 6×2 [lower, upper] in radians

fig = figure('Name', ['Path: ' algorithm_name], 'NumberTitle', 'off', ...
             'Position', [100, 100, 1000, 500]);

joint_labels = {'Joint 1 (base)', 'Joint 2 (shoulder)', 'Joint 3 (elbow)', ...
                'Joint 4 (wrist roll)', 'Joint 5 (wrist pitch)', 'Joint 6 (flange)'};

n_pts = size(path, 1);
x_vec = 1:n_pts;

for j = 1:6
    subplot(2, 3, j);

    % Trajectory
    plot(x_vec, path(:, j), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Path');
    hold on;

    % Joint limits as horizontal dashed red lines
    yline(limits(j, 1), 'r--', 'LineWidth', 0.8, 'Label', ...
          sprintf('%.1f°', rad2deg(limits(j,1))), 'LabelHorizontalAlignment', 'right');
    yline(limits(j, 2), 'r--', 'LineWidth', 0.8, 'Label', ...
          sprintf('%.1f°', rad2deg(limits(j,2))), 'LabelHorizontalAlignment', 'right');

    % Shade the valid joint range lightly
    y_range = [limits(j,1), limits(j,2)];
    fill([0, n_pts, n_pts, 0], [y_range(1), y_range(1), y_range(2), y_range(2)], ...
         [0.9, 0.95, 1.0], 'FaceAlpha', 0.20, 'EdgeColor', 'none', ...
         'HandleVisibility', 'off');

    xlabel('Waypoint index', 'FontSize', 8);
    ylabel('[rad]', 'FontSize', 8);
    title(joint_labels{j}, 'FontSize', 9);
    grid on;
    xlim([1, n_pts]);

    % Auto-expand y limits slightly beyond the joint range for visibility
    ylim([limits(j,1) - 0.1, limits(j,2) + 0.1]);

    hold off;
end

sgtitle([algorithm_name ' — Joint space path (' num2str(n_pts) ' waypoints)'], ...
        'FontSize', 11, 'FontWeight', 'bold');

end
