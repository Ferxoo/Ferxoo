%% ─────────────────────────────────────────────────────────────
% File        : draw_rrtstar_tree.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Visualises the RRT* exploration tree in Cartesian EE
%               space (Fig 4.7). Requires plan_rrt_star to have been
%               called with opts.return_tree = true so that info contains
%               .tree_nodes (N×6) and .tree_parents (N×1).
%               Plots: tree edges (grey), final path edges (orange),
%               obstacles (translucent), start/goal EE markers.
%               Saved as results/figures/rrtstar_tree.png (Fig 4.7).
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               path      — M×6 double, final planned path [rad]
%               info      — struct from plan_rrt_star with return_tree=true
%               obstacles — cell array of obstacle structs
% Outputs     : none (figure saved as side-effect)
% Dependencies: getTransform (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function draw_rrtstar_tree(robot, path, info, obstacles)

narginchk(4, 4);

if ~isfield(info, 'tree_nodes') || ~isfield(info, 'tree_parents')
    warning(['draw_rrtstar_tree: info does not contain tree_nodes/tree_parents. ' ...
             'Call plan_rrt_star with opts.return_tree=true.']);
    return;
end

nodes   = info.tree_nodes;    % N×6
parents = info.tree_parents;  % N×1
N       = size(nodes, 1);

%% Subsample tree nodes for EE FK (max 600 to keep it fast) ────
idx_tree = unique(round(linspace(1, N, min(N, 600))));
ee_tree  = zeros(numel(idx_tree), 3);
fprintf('[draw_rrtstar_tree] Computing FK for %d tree nodes...\n', numel(idx_tree));
for k = 1:numel(idx_tree)
    T = getTransform(robot, nodes(idx_tree(k),:)', robot.BodyNames{end});
    ee_tree(k,:) = T(1:3,4)';
end

%% EE positions along the final path ──────────────────────────
n_path  = size(path,1);
ee_path = zeros(n_path,3);
for k = 1:n_path
    T = getTransform(robot, path(k,:)', robot.BodyNames{end});
    ee_path(k,:) = T(1:3,4)';
end

%% Build parent map for subsampled nodes ──────────────────────
% Build tree edges among the subsampled set by tracing parents
idx_set = idx_tree;
edge_src = [];
edge_dst = [];
for k = 1:numel(idx_set)
    ni = idx_set(k);
    pi = parents(ni);
    if pi == 0, continue; end
    % Find nearest available parent in subsampled set
    [~, pi_sub] = min(abs(idx_set - pi));
    T_k  = getTransform(robot, nodes(ni,:)', robot.BodyNames{end});
    T_pi = getTransform(robot, nodes(idx_set(pi_sub),:)', robot.BodyNames{end});
    edge_src(end+1,:) = T_pi(1:3,4)';  %#ok<AGROW>
    edge_dst(end+1,:) = T_k(1:3,4)';   %#ok<AGROW>
end

%% Figure ─────────────────────────────────────────────────────
fig = figure('Name', 'RRT* Tree — Scenario 2', ...
             'NumberTitle', 'off', 'Position', [100, 100, 960, 720]);

hold on;

% Draw tree edges
for k = 1:size(edge_src,1)
    plot3([edge_src(k,1), edge_dst(k,1)], ...
          [edge_src(k,2), edge_dst(k,2)], ...
          [edge_src(k,3), edge_dst(k,3)], ...
          '-', 'Color', [0.78,0.78,0.78], 'LineWidth', 0.6, ...
          'HandleVisibility', 'off');
end

% Draw tree node dots (subsampled)
plot3(ee_tree(:,1), ee_tree(:,2), ee_tree(:,3), '.', ...
      'Color', [0.70,0.70,0.70], 'MarkerSize', 3, 'DisplayName', 'Tree nodes');

% Draw obstacles
for k = 1:numel(obstacles)
    obs = obstacles{k};
    T   = obs.T_world;
    clr = obs.color;
    if isa(obs.geometry, 'collisionBox')
        drawBox(T, [obs.geometry.X, obs.geometry.Y, obs.geometry.Z], clr, 0.40);
    elseif isa(obs.geometry, 'collisionCylinder')
        drawCylinder(T, obs.geometry.Radius, obs.geometry.Length, clr, 0.40);
    elseif isa(obs.geometry, 'collisionSphere')
        [sx,sy,sz] = sphere(14);
        r = obs.geometry.Radius;  c = T(1:3,4)';
        surf(r*sx+c(1), r*sy+c(2), r*sz+c(3), 'FaceColor', clr, ...
             'FaceAlpha', 0.40, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
end

% Draw final path in orange (thick)
plot3(ee_path(:,1), ee_path(:,2), ee_path(:,3), '-', ...
      'Color', [1.00, 0.45, 0.00], 'LineWidth', 3.0, 'DisplayName', 'Final path');

% Start and goal markers
plot3(ee_path(1,1), ee_path(1,2), ee_path(1,3), 'go', ...
      'MarkerSize', 14, 'MarkerFaceColor', [0.1,0.75,0.2], 'LineWidth', 2, ...
      'DisplayName', 'EE start');
plot3(ee_path(end,1), ee_path(end,2), ee_path(end,3), 'r*', ...
      'MarkerSize', 14, 'LineWidth', 2, 'DisplayName', 'EE goal');

axis equal;  grid on;
xlabel('X [m]');  ylabel('Y [m]');  zlabel('Z [m]');
view(35, 22);
xlim([-0.20, 0.80]);  ylim([-0.50, 0.50]);  zlim([-0.05, 1.00]);
campos([2.2, -1.8, 1.6]);  camtarget([0.30, 0, 0.45]);  camup([0,0,1]);  camva(20);

legend('Location', 'northeast', 'FontSize', 10);
title({sprintf('RRT* Exploration Tree — Scenario 2  (%d nodes)', N), ...
       sprintf('Iterations: %d | Path length: %.3f rad | Time: %.2f s', ...
               info.num_iterations, info.path_length, info.computation_time)}, ...
      'FontSize', 11);
hold off;

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'rrtstar_tree');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_rrtstar_tree] Figure saved: %s.png\n', out_base);

end

%% ── Drawing helpers ──────────────────────────────────────────

function drawBox(T, dims, color, alpha_val)
    hx=dims(1)/2; hy=dims(2)/2; hz=dims(3)/2;
    v=[-hx -hy -hz; hx -hy -hz; hx hy -hz; -hx hy -hz;
       -hx -hy  hz; hx -hy  hz; hx hy  hz; -hx hy  hz];
    vw=(T(1:3,1:3)*v'+repmat(T(1:3,4),1,8))';
    f=[1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 2 3 7 6; 1 4 8 5];
    patch('Vertices',vw,'Faces',f,'FaceColor',color,'FaceAlpha',alpha_val,'EdgeColor','none');
end

function drawCylinder(T, radius, height, color, alpha_val)
    [cx,cy,cz]=cylinder(radius,24); cz=cz*height-height/2; sz=size(cx);
    pts=[cx(:)';cy(:)';cz(:)';ones(1,numel(cx))]; pts_w=T*pts;
    cx_w=reshape(pts_w(1,:),sz); cy_w=reshape(pts_w(2,:),sz); cz_w=reshape(pts_w(3,:),sz);
    surf(cx_w,cy_w,cz_w,'FaceColor',color,'FaceAlpha',alpha_val,'EdgeColor','none');
    fill3(cx_w(1,:),cy_w(1,:),cz_w(1,:),color,'FaceAlpha',alpha_val,'EdgeColor','none');
    fill3(cx_w(2,:),cy_w(2,:),cz_w(2,:),color,'FaceAlpha',alpha_val,'EdgeColor','none');
end
