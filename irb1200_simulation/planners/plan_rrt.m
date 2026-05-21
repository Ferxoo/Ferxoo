%% ─────────────────────────────────────────────────────────────
% File        : plan_rrt.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox (NO Navigation Toolbox)
% Description : Rapidly-exploring Random Tree (RRT) planner in the 6-DOF
%               joint space. Pure MATLAB implementation using getTransform
%               for conservative sphere-based collision checking.
%               Reference: LaValle, S.M. (1998). Rapidly-exploring random
%               trees: A new tool for path planning. TR 98-11, Iowa State.
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               q_start   — 6×1 double [rad]
%               q_goal    — 6×1 double [rad]
%               obstacles — cell array of obstacle structs
%               opts      — struct with optional fields:
%                  .max_iterations (default 5000)
%                  .goal_bias      (default 0.05)
%                  .max_conn_dist  (default 0.20 [rad])
%                  .val_step       (default 0.05 [rad], edge check step)
%                  .timeout        (default 15.0 [s])
% Outputs     : path — N×6 double [rad];  info — result struct
% Dependencies: get_joint_limits, getTransform (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function [path, info] = plan_rrt(robot, q_start, q_goal, obstacles, opts)

narginchk(4, 5);
if nargin < 5 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'max_iterations', 5000, ...
    'goal_bias',      0.05, ...
    'max_conn_dist',  0.20, ...
    'val_step',       0.05, ...
    'timeout',        15.0));

q_start = q_start(:);  q_goal = q_goal(:);
assert(numel(q_start)==6, 'plan_rrt: q_start must be 6×1.');
assert(numel(q_goal) ==6, 'plan_rrt: q_goal must be 6×1.');

limits = get_joint_limits();   % 6×2 [lower, upper]

%% Tree initialisation ─────────────────────────────────────────
nodes   = q_start';    % N×6 matrix — one row per node
parents = 0;           % parent index per node (0 = root, no parent)

success = false;
t_start = tic;
iter    = 0;

%% RRT main loop ───────────────────────────────────────────────
for iter = 1:opts.max_iterations

    %% 1. Sample random configuration (with goal bias)
    if rand < opts.goal_bias
        q_rand = q_goal';
    else
        q_rand = limits(:,1)' + rand(1,6) .* (limits(:,2)-limits(:,1))';
    end

    %% 2. Find nearest node in the tree
    dists        = vecnorm(nodes - repmat(q_rand, size(nodes,1), 1), 2, 2);
    [d_near, ni] = min(dists);
    q_near       = nodes(ni,:);

    %% 3. Extend toward q_rand (limited by max_conn_dist)
    if d_near > opts.max_conn_dist
        q_new = q_near + opts.max_conn_dist * (q_rand - q_near) / d_near;
    else
        q_new = q_rand;
    end
    % Clamp to joint limits
    q_new = max(limits(:,1)', min(limits(:,2)', q_new));

    %% 4. Collision-check the new edge; add node if free
    if isPathFree(robot, q_near, q_new, obstacles, opts.val_step)
        nodes(end+1,:) = q_new;
        parents(end+1) = ni;

        %% 5. Check proximity to goal and attempt direct connection
        if norm(q_new - q_goal') <= opts.max_conn_dist && ...
           isPathFree(robot, q_new, q_goal', obstacles, opts.val_step)
            nodes(end+1,:) = q_goal';
            parents(end+1) = size(nodes,1) - 1;
            success = true;
            break;
        end
    end

    %% Timeout guard
    if toc(t_start) > opts.timeout
        break;
    end
end

elapsed = toc(t_start);

%% Extract path ────────────────────────────────────────────────
if success
    path = tracePath(nodes, parents, size(nodes,1));
else
    path = zeros(0, 6);
end

info.success          = success;
info.computation_time = elapsed;
info.num_iterations   = iter;
info.path_length      = ternary(success && size(path,1)>1, ...
                                sum(vecnorm(diff(path),2,2)), NaN);
end

%% ══════════════════════════════════════════════════════════════
%% Shared local helpers (duplicated across plan_rrt*.m files to
%% keep each file self-contained and independently runnable)
%% ══════════════════════════════════════════════════════════════

function free = isPathFree(robot, q1, q2, obstacles, step)
% Returns true if the straight-line joint-space edge from q1 to q2 is
% free of collisions. Checks at uniform intervals of 'step' radians.
    if isempty(obstacles), free = true; return; end
    d = norm(q2 - q1);
    if d < 1e-8, free = isConfigFree(robot, q1, obstacles); return; end
    n = max(2, ceil(d / step));
    for k = 0:n
        q_k = q1 + (k/n) * (q2 - q1);
        if ~isConfigFree(robot, q_k, obstacles)
            free = false;  return;
        end
    end
    free = true;
end

function free = isConfigFree(robot, q, obstacles)
    if isempty(obstacles), free = true; return; end
    is_sph    = cellfun(@(o) isa(o.geometry, 'collisionSphere'), obstacles);
    sph_obs   = obstacles( is_sph);
    other_obs = obstacles(~is_sph);
    if ~isempty(sph_obs) && ~isConfigFree_spheres(robot, q(:), sph_obs)
        free = false; return;
    end
    if ~isempty(other_obs)
        geoms = cellfun(@(o) o.geometry, other_obs, 'UniformOutput', false);
        try
            free = ~checkCollision(robot, q(:), geoms, 'IgnoreSelfCollision', 'on');
        catch
            free = true;
        end
    else
        free = true;
    end
end

function free = isConfigFree_spheres(robot, q, sphere_obs)
    LINK_NAMES = {'link_1','link_2','link_3','link_4','link_5','link_6','tool0'};
    LINK_RADII = [0.055,   0.055,   0.050,   0.040,   0.038,   0.030,  0.020];
    MARGIN     = 0.015;
    free = true;
    for oi = 1:numel(sphere_obs)
        obs   = sphere_obs{oi};
        r_obs = obs.geometry.Radius;
        c_obs = obs.T_world(1:3, 4);
        for li = 1:numel(LINK_NAMES)
            T_li = getTransform(robot, q(:), LINK_NAMES{li});
            if norm(T_li(1:3,4) - c_obs) < (r_obs + LINK_RADII(li) + MARGIN)
                free = false; return;
            end
        end
    end
end

function r = obstacleRadius(obs)
    if     isa(obs.geometry,'collisionBox')
        r = 0.5 * norm([obs.geometry.X, obs.geometry.Y, obs.geometry.Z]);
    elseif isa(obs.geometry,'collisionCylinder')
        r = max(obs.geometry.Radius, obs.geometry.Length * 0.5);
    elseif isa(obs.geometry,'collisionSphere')
        r = obs.geometry.Radius;
    else
        r = 0.10;
    end
end

function path = tracePath(nodes, parents, leaf_idx)
% Trace from leaf back to root and return the ordered N×6 path.
    idx     = leaf_idx;
    indices = leaf_idx;
    while parents(idx) ~= 0
        idx     = parents(idx);
        indices = [idx, indices];  %#ok<AGROW>
    end
    path = nodes(indices, :);
end

function opts = applyDefaults(opts, defaults)
    fields = fieldnames(defaults);
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(opts, f), opts.(f) = defaults.(f); end
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
