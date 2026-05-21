%% ─────────────────────────────────────────────────────────────
% File        : draw_distance_scheme.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 1024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Illustrates the surface-to-surface distance calculation
%               used in compute_min_distance. Left panel shows the robot
%               in a reference configuration with the 7 link-origin spheres
%               (used for conservative collision approximation) and the
%               operator sphere overlaid. Right panel zooms in on a single
%               link-operator pair and annotates the centre-to-centre
%               distance, sphere radii and surface-to-surface gap.
%               Saved as results/figures/distance_scheme.png (Fig 4.4).
% Inputs      : robot — rigidBodyTree (DataFormat='column')
% Outputs     : none (figure saved as side-effect)
% Dependencies: getTransform (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function draw_distance_scheme(robot)

narginchk(1, 1);

% Reference configuration: elbow mid-range, arm pointing forward-ish
q_ref = deg2rad([0; -45; 45; 0; 45; 0]);

% Link names, sphere radii and colours matching compute_min_distance.m
LINK_NAMES  = {'link_1','link_2','link_3','link_4','link_5','link_6','tool0'};
LINK_RADII  = [0.055,   0.055,   0.050,   0.040,   0.038,   0.030,  0.020];
LINK_COLORS = [0.20,0.50,0.80; 0.20,0.70,0.50; 1.00,0.65,0.00; ...
               0.80,0.20,0.80; 0.20,0.80,0.80; 0.60,0.60,0.60; ...
               0.90,0.30,0.10];

% Operator position and radius
op_pos = [0.70, 0.0, 0.55];
R_OP   = 0.150;

%% Compute link origins in world frame ─────────────────────────
link_pos = zeros(numel(LINK_NAMES), 3);
for k = 1:numel(LINK_NAMES)
    T_k         = getTransform(robot, q_ref, LINK_NAMES{k});
    link_pos(k,:) = T_k(1:3,4)';
end

fig = figure('Name', 'Distance Calculation Scheme — compute_min_distance', ...
             'NumberTitle', 'off', 'Position', [100, 100, 1150, 540]);

%% ── Left panel: full robot with link spheres ──────────────────
ax1 = subplot(1, 2, 1);
show(robot, q_ref, 'Frames', 'off', 'PreservePlot', false, 'Parent', ax1);
patches = findobj(ax1, 'Type', 'Patch');
set(patches, 'FaceColor', [0.65, 0.65, 0.65], 'FaceAlpha', 0.55);
hold(ax1, 'on');

% Draw link-origin spheres
[sx, sy, sz] = sphere(14);
for k = 1:numel(LINK_NAMES)
    r  = LINK_RADII(k);
    c  = link_pos(k,:);
    cl = LINK_COLORS(k,:);
    surf(ax1, r*sx+c(1), r*sy+c(2), r*sz+c(3), ...
         'FaceColor', cl, 'FaceAlpha', 0.55, 'EdgeColor', 'none', ...
         'DisplayName', LINK_NAMES{k});
    plot3(ax1, c(1), c(2), c(3), 'k.', 'MarkerSize', 6, ...
          'HandleVisibility', 'off');
end

% Draw operator sphere
surf(ax1, R_OP*sx+op_pos(1), R_OP*sy+op_pos(2), R_OP*sz+op_pos(3), ...
     'FaceColor', [0.20,0.40,0.90], 'FaceAlpha', 0.50, 'EdgeColor', 'none', ...
     'DisplayName', sprintf('Operator (R=%.0f mm)', R_OP*1000));
plot3(ax1, op_pos(1), op_pos(2), op_pos(3), 'b.', 'MarkerSize', 8, ...
      'HandleVisibility', 'off');

axis(ax1,'equal');  grid(ax1,'on');
xlabel(ax1,'X [m]');  ylabel(ax1,'Y [m]');  zlabel(ax1,'Z [m]');
view(ax1, 35, 20);
xlim(ax1,[-0.25, 1.10]);  ylim(ax1,[-0.50, 0.50]);  zlim(ax1,[-0.05, 1.00]);
title(ax1, {'Link-origin spheres and operator sphere', ...
            '(7 reference points, radii 20–55 mm)'}, 'FontSize', 11);
legend(ax1, 'Location', 'northeast', 'FontSize', 7, 'NumColumns', 2);
hold(ax1, 'off');

%% ── Right panel: annotated distance diagram for tool0 ─────────
% Show closest link (tool0) vs operator with distance annotation
ax2 = subplot(1, 2, 2);
hold(ax2, 'on');

% Find closest link to operator
dists_c = vecnorm(link_pos - repmat(op_pos, numel(LINK_NAMES), 1), 2, 2);
[~, ci]  = min(dists_c);
p_link   = link_pos(ci,:);
r_link   = LINK_RADII(ci);
c_link   = LINK_COLORS(ci,:);

% Draw the two spheres in 2D (YZ plane projection or XZ)
theta = linspace(0, 2*pi, 120);

% Link sphere (2D circle, XZ projection)
lx = p_link(1) + r_link * cos(theta);
lz = p_link(3) + r_link * sin(theta);
fill(ax2, lx, lz, c_link, 'FaceAlpha', 0.50, 'EdgeColor', c_link*0.7, ...
     'LineWidth', 1.5, 'DisplayName', sprintf('%s (r=%.0f mm)', ...
     LINK_NAMES{ci}, r_link*1000));
plot(ax2, p_link(1), p_link(3), 'k+', 'MarkerSize', 10, 'LineWidth', 1.5, ...
     'HandleVisibility', 'off');

% Operator sphere
ox = op_pos(1) + R_OP * cos(theta);
oz = op_pos(3) + R_OP * sin(theta);
fill(ax2, ox, oz, [0.20,0.40,0.90], 'FaceAlpha', 0.45, 'EdgeColor', [0,0,0.7], ...
     'LineWidth', 1.5, 'DisplayName', sprintf('Operator (R=%.0f mm)', R_OP*1000));
plot(ax2, op_pos(1), op_pos(3), 'b+', 'MarkerSize', 10, 'LineWidth', 1.5, ...
     'HandleVisibility', 'off');

% Centre-to-centre line
d_cc = norm(op_pos - p_link);
plot(ax2, [p_link(1), op_pos(1)], [p_link(3), op_pos(3)], 'k--', ...
     'LineWidth', 1.2, 'DisplayName', sprintf('d_{cc} = %.3f m', d_cc));

% Surface-to-surface gap annotation
d_ss = max(d_cc - r_link - R_OP, 0);
unit_v = (op_pos - p_link) / d_cc;
p_surf_link = p_link + r_link * unit_v;
p_surf_op   = op_pos - R_OP   * unit_v;
plot(ax2, [p_surf_link(1), p_surf_op(1)], [p_surf_link(3), p_surf_op(3)], ...
     'r-', 'LineWidth', 2.5, 'DisplayName', ...
     sprintf('d_{surf} = %.3f m', d_ss));

% Annotate radii with braces
mid_x = (p_link(1) + op_pos(1)) / 2;
mid_z = (p_link(3) + op_pos(3)) / 2;
text(ax2, mid_x, mid_z + 0.04, sprintf('d_{surf} = %.0f mm', d_ss*1000), ...
     'FontSize', 10, 'Color', [0.8,0,0], 'HorizontalAlignment', 'center', ...
     'FontWeight', 'bold');
text(ax2, p_link(1), p_link(3) - r_link - 0.02, ...
     sprintf('r_{link} = %.0f mm', r_link*1000), ...
     'FontSize', 9, 'Color', c_link*0.8, 'HorizontalAlignment', 'center');
text(ax2, op_pos(1), op_pos(3) - R_OP - 0.02, ...
     sprintf('R_{OP} = %.0f mm', R_OP*1000), ...
     'FontSize', 9, 'Color', [0,0,0.7], 'HorizontalAlignment', 'center');

axis(ax2, 'equal');  grid(ax2, 'on');
xlabel(ax2, 'X [m]', 'FontSize', 11);
ylabel(ax2, 'Z [m]', 'FontSize', 11);
title(ax2, {'Surface-to-surface distance d_{surf}', ...
            'd_{surf} = d_{cc} \minus r_{link} \minus R_{OP}'}, 'FontSize', 11);
legend(ax2, 'Location', 'northwest', 'FontSize', 9);
hold(ax2, 'off');

sgtitle('compute\_min\_distance: Link-sphere Distance Calculation (ISO/TS 15066 SSM)', ...
        'FontSize', 12, 'FontWeight', 'bold');

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'distance_scheme');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_distance_scheme] Figure saved: %s.png\n', out_base);

end
