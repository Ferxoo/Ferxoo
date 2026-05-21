%% ─────────────────────────────────────────────────────────────
% File        : draw_apf_localmin.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Illustrates an APF local-minimum scenario (Fig 4.6).
%               Runs plan_apf on Scenario 4 (dense obstacles) from the
%               standard benchmark configs. If the planner fails (stuck),
%               plots the stuck joint-space trajectory in Cartesian EE
%               space alongside the obstacles and the goal marker.
%               If APF unexpectedly succeeds the full path is shown.
%               Saved as results/figures/apf_localmin.png (Fig 4.6).
% Inputs      : robot — rigidBodyTree (DataFormat='column')
% Outputs     : none (figure saved as side-effect)
% Dependencies: plan_apf, build_scenario, getTransform
%% ─────────────────────────────────────────────────────────────

function draw_apf_localmin(robot)

narginchk(1, 1);

% Standard benchmark configurations (same as main_simulation.m)
q_start = deg2rad([-90; 70; 0; 0; 0; 0]);
q_goal  = deg2rad([ 90; 70; 0; 0; 0; 0]);

% Scenario 4 — dense, most likely to cause APF local minima
obstacles = build_scenario(4);

fprintf('[draw_apf_localmin] Running APF on Scenario 4...\n');
[path_apf, info_apf] = plan_apf(robot, q_start, q_goal, obstacles, struct());

if info_apf.success
    fprintf('[draw_apf_localmin] APF succeeded in %.2f s (%d iter). Plotting path.\n', ...
            info_apf.computation_time, info_apf.num_iterations);
    title_str = 'APF — Path found (no local min in this run)';
else
    fprintf('[draw_apf_localmin] APF stuck after %d iter (%.2f s). Plotting stuck trajectory.\n', ...
            info_apf.num_iterations, info_apf.computation_time);
    title_str = 'APF Local Minimum — Trajectory stalled in Scenario 4';
end

%% Forward kinematics of EE along APF trajectory ───────────────
n_wp   = size(path_apf, 1);
ee_pos = zeros(n_wp, 3);
% Subsample to at most 300 points for speed
idx_sub = unique(round(linspace(1, n_wp, min(n_wp, 300))));
for k = 1:numel(idx_sub)
    T_ee          = getTransform(robot, path_apf(idx_sub(k),:)', robot.BodyNames{end});
    ee_pos(k,:)   = T_ee(1:3,4)';
end
ee_pos = ee_pos(1:numel(idx_sub),:);

%% EE of start and goal ────────────────────────────────────────
T_start = getTransform(robot, q_start, robot.BodyNames{end});
T_goal  = getTransform(robot, q_goal,  robot.BodyNames{end});
ee_s    = T_start(1:3,4)';
ee_g    = T_goal(1:3,4)';

%% Figure ─────────────────────────────────────────────────────
fig = figure('Name', 'APF Local Minimum — Scenario 4', ...
             'NumberTitle', 'off', 'Position', [100, 100, 950, 700]);

%% Robot at final (stuck) configuration ───────────────────────
q_final = path_apf(end,:)';
show(robot, q_final, 'Frames', 'off', 'PreservePlot', false);
robot_patches = findobj(gca, 'Type', 'Patch');
set(robot_patches, 'FaceColor', [0.90, 0.30, 0.10], 'FaceAlpha', 0.70);
hold on;

% Draw obstacles using same helpers as draw_scene
for k = 1:numel(obstacles)
    obs = obstacles{k};
    T   = obs.T_world;
    clr = obs.color;
    if isa(obs.geometry, 'collisionBox')
        dims = [obs.geometry.X, obs.geometry.Y, obs.geometry.Z];
        drawBox(T, dims, clr, 0.50);
    elseif isa(obs.geometry, 'collisionCylinder')
        drawCylinder(T, obs.geometry.Radius, obs.geometry.Length, clr, 0.50);
    elseif isa(obs.geometry, 'collisionSphere')
        [sx,sy,sz] = sphere(14);
        r = obs.geometry.Radius;
        c = T(1:3,4)';
        surf(r*sx+c(1), r*sy+c(2), r*sz+c(3), 'FaceColor', clr, ...
             'FaceAlpha', 0.50, 'EdgeColor', 'none');
    end
end

% EE trajectory (colour-coded: start green → stuck/goal red)
n_plot = size(ee_pos,1);
cmap   = [linspace(0.2,0.9,n_plot)', linspace(0.7,0.1,n_plot)', ...
           linspace(0.2,0.1,n_plot)'];
for k = 1:n_plot-1
    plot3(ee_pos(k:k+1,1), ee_pos(k:k+1,2), ee_pos(k:k+1,3), '-', ...
          'Color', cmap(k,:), 'LineWidth', 1.5, 'HandleVisibility', 'off');
end

% Start and goal EE markers
plot3(ee_s(1), ee_s(2), ee_s(3), 'go', 'MarkerSize', 12, ...
      'MarkerFaceColor', [0.1,0.75,0.2], 'LineWidth', 2, 'DisplayName', 'EE start');
plot3(ee_g(1), ee_g(2), ee_g(3), 'r*', 'MarkerSize', 12, 'LineWidth', 2, ...
      'DisplayName', 'EE goal');
if ~info_apf.success
    plot3(ee_pos(end,1), ee_pos(end,2), ee_pos(end,3), 'rs', ...
          'MarkerSize', 12, 'MarkerFaceColor', [1,0.4,0.4], 'LineWidth', 2, ...
          'DisplayName', 'EE final (stuck)');
end

axis equal;  grid on;
xlabel('X [m]');  ylabel('Y [m]');  zlabel('Z [m]');
view(35, 22);
xlim([-0.15, 0.75]);  ylim([-0.50, 0.50]);  zlim([-0.05, 0.95]);
campos([2.0, -1.8, 1.5]);  camtarget([0.35, 0, 0.40]);  camup([0,0,1]);  camva(20);

legend('Location', 'northeast', 'FontSize', 10);
title({title_str, ...
       sprintf('Iterations: %d | Success: %s | Time: %.2f s', ...
               info_apf.num_iterations, mat2str(info_apf.success), ...
               info_apf.computation_time)}, 'FontSize', 11);
hold off;

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'apf_localmin');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_apf_localmin] Figure saved: %s.png\n', out_base);

end

%% ── Drawing helpers (same style as draw_scene.m) ─────────────

function drawBox(T, dims, color, alpha_val)
    hx = dims(1)/2;  hy = dims(2)/2;  hz = dims(3)/2;
    v = [-hx -hy -hz; hx -hy -hz; hx  hy -hz; -hx  hy -hz;
         -hx -hy  hz; hx -hy  hz; hx  hy  hz; -hx  hy  hz];
    vw = (T(1:3,1:3)*v' + repmat(T(1:3,4),1,8))';
    f  = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 2 3 7 6; 1 4 8 5];
    patch('Vertices',vw,'Faces',f,'FaceColor',color,'FaceAlpha',alpha_val,'EdgeColor','none');
end

function drawCylinder(T, radius, height, color, alpha_val)
    [cx,cy,cz] = cylinder(radius, 24);
    cz = cz*height - height/2;
    sz = size(cx);
    pts   = [cx(:)'; cy(:)'; cz(:)'; ones(1,numel(cx))];
    pts_w = T*pts;
    cx_w  = reshape(pts_w(1,:),sz);
    cy_w  = reshape(pts_w(2,:),sz);
    cz_w  = reshape(pts_w(3,:),sz);
    surf(cx_w,cy_w,cz_w,'FaceColor',color,'FaceAlpha',alpha_val,'EdgeColor','none');
    fill3(cx_w(1,:),cy_w(1,:),cz_w(1,:),color,'FaceAlpha',alpha_val,'EdgeColor','none');
    fill3(cx_w(2,:),cy_w(2,:),cz_w(2,:),color,'FaceAlpha',alpha_val,'EdgeColor','none');
end
