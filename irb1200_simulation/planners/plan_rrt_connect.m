%% ─────────────────────────────────────────────────────────────
% File        : plan_rrt_connect.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox (NO Navigation Toolbox)
% Description : Bidirectional RRT (RRT-Connect) planner. Grows two trees
%               simultaneously — one from start, one from goal — and uses
%               a greedy CONNECT step to merge them. Typically 2-3× faster
%               than unidirectional RRT on single-query problems.
%               Reference: Kuffner, J.J. & LaValle, S.M. (2000).
%               RRT-Connect: An efficient approach to single-query path
%               planning. ICRA 2000.
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               q_start   — 6×1 double [rad]
%               q_goal    — 6×1 double [rad]
%               obstacles — cell array of obstacle structs
%               opts      — struct with optional fields:
%                  .max_iterations (default 3000)
%                  .goal_bias      (default 0.05)
%                  .max_conn_dist  (default 0.20 [rad])
%                  .val_step       (default 0.05 [rad])
%                  .timeout        (default 10.0 [s])
% Outputs     : path — N×6 double [rad];  info — result struct
% Dependencies: get_joint_limits, getTransform (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function [path, info] = plan_rrt_connect(robot, q_start, q_goal, obstacles, opts)

narginchk(4, 5);
if nargin < 5 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'max_iterations', 3000, ...
    'goal_bias',      0.05, ...
    'max_conn_dist',  0.20, ...
    'val_step',       0.05, ...
    'timeout',        10.0));

q_start = q_start(:);  q_goal = q_goal(:);
assert(numel(q_start)==6, 'plan_rrt_connect: q_start must be 6×1.');
assert(numel(q_goal) ==6, 'plan_rrt_connect: q_goal must be 6×1.');

limits = get_joint_limits();

%% Two-tree initialisation ─────────────────────────────────────
% Tree A grows from start; Tree B grows from goal.
nodesA   = q_start';   parentsA = 0;
nodesB   = q_goal';    parentsB = 0;

success   = false;
midxA     = 0;    % meeting node index in tree A
midxB     = 0;    % meeting node index in tree B

t_start = tic;
iter    = 0;

%% BiRRT main loop ─────────────────────────────────────────────
for iter = 1:opts.max_iterations

    %% Sample random config (uniform; no goal bias for BiRRT)
    q_rand = limits(:,1)' + rand(1,6) .* (limits(:,2)-limits(:,1))';

    %% Extend tree A toward q_rand
    [nodesA, parentsA, newA] = extendTree(nodesA, parentsA, q_rand, ...
                                          robot, obstacles, opts, limits);
    if newA > 0
        % CONNECT step: try to reach the new A node from the nearest B node
        q_new_A = nodesA(newA,:);
        [close_B, dB] = nearestIdx(nodesB, q_new_A);

        if dB <= opts.max_conn_dist && ...
           isPathFree(robot, nodesB(close_B,:), q_new_A, obstacles, opts.val_step)
            midxA = newA;  midxB = close_B;
            success = true;  break;
        end
    end

    %% Extend tree B toward q_rand (swap roles each iteration)
    [nodesB, parentsB, newB] = extendTree(nodesB, parentsB, q_rand, ...
                                          robot, obstacles, opts, limits);
    if newB > 0
        % CONNECT step: try to reach the new B node from the nearest A node
        q_new_B = nodesB(newB,:);
        [close_A, dA] = nearestIdx(nodesA, q_new_B);

        if dA <= opts.max_conn_dist && ...
           isPathFree(robot, nodesA(close_A,:), q_new_B, obstacles, opts.val_step)
            midxA = close_A;  midxB = newB;
            success = true;  break;
        end
    end

    if toc(t_start) > opts.timeout,  break;  end
end

elapsed = toc(t_start);

%% Assemble bidirectional path ─────────────────────────────────
if success
    pathA    = tracePath(nodesA, parentsA, midxA);   % start → meeting
    pathB    = tracePath(nodesB, parentsB, midxB);   % goal  → meeting → reverse
    raw_path = [pathA; flip(pathB, 1)];              % start → meeting → goal
    path     = shortcutPath(raw_path, robot, obstacles, opts.val_step);
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
%% Local helpers
%% ══════════════════════════════════════════════════════════════

function [nodes, parents, new_idx] = extendTree(nodes, parents, q_target, ...
                                                 robot, obstacles, opts, limits)
% Extend the tree one step toward q_target; returns new node index (0 if blocked).
    [ni, ~]  = nearestIdx(nodes, q_target);
    q_near   = nodes(ni,:);
    d        = norm(q_target - q_near);
    if d > opts.max_conn_dist
        q_new = q_near + opts.max_conn_dist * (q_target - q_near) / d;
    else
        q_new = q_target;
    end
    q_new = max(limits(:,1)', min(limits(:,2)', q_new));

    if norm(q_new - q_near) < 1e-8 || ...
       ~isPathFree(robot, q_near, q_new, obstacles, opts.val_step)
        new_idx = 0;
        return;
    end
    nodes(end+1,:) = q_new;
    parents(end+1) = ni;
    new_idx        = size(nodes, 1);
end

function [idx, d] = nearestIdx(nodes, q)
    dists = vecnorm(nodes - repmat(q, size(nodes,1), 1), 2, 2);
    [d, idx] = min(dists);
end

function free = isPathFree(robot, q1, q2, obstacles, step)
    if isempty(obstacles), free = true; return; end
    d = norm(q2 - q1);
    if d < 1e-8, free = isConfigFree(robot, q1, obstacles); return; end
    n = max(2, ceil(d / step));
    for k = 0:n
        if ~isConfigFree(robot, q1 + (k/n)*(q2-q1), obstacles)
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
        r = 0.5*norm([obs.geometry.X, obs.geometry.Y, obs.geometry.Z]);
    elseif isa(obs.geometry,'collisionCylinder')
        r = max(obs.geometry.Radius, obs.geometry.Length*0.5);
    elseif isa(obs.geometry,'collisionSphere')
        r = obs.geometry.Radius;
    else
        r = 0.10;
    end
end

function path = tracePath(nodes, parents, leaf_idx)
    idx = leaf_idx;  indices = leaf_idx;
    while parents(idx) ~= 0
        idx = parents(idx);
        indices = [idx, indices];  %#ok<AGROW>
    end
    path = nodes(indices, :);
end

function opts = applyDefaults(opts, defaults)
    fields = fieldnames(defaults);
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(opts,f), opts.(f) = defaults.(f); end
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function path = shortcutPath(raw_path, robot, obstacles, val_step)
% Greedy O(N²) shortcutting with up to 15-node lookahead.
% Iteratively tries to replace the segment i→i+1→…→j with a direct edge i→j.
% A shortcut is accepted only when the straight joint-space segment is
% collision-free. Multiple passes are performed until no improvement is found.
    path    = raw_path;
    changed = true;
    while changed
        changed = false;
        i = 1;
        while i < size(path, 1) - 1
            max_j = min(size(path, 1), i + 15);
            for j = max_j:-1:(i+2)
                if isSegmentFree(path(i,:), path(j,:), robot, obstacles, val_step)
                    path    = [path(1:i,:); path(j:end,:)];
                    changed = true;
                    break;
                end
            end
            i = i + 1;
        end
    end
end

function free = isSegmentFree(q1, q2, robot, obstacles, step)
% Returns true when the straight joint-space edge from q1 to q2 is free
% of collisions, checked at uniform intervals of 'step' radians.
    if isempty(obstacles), free = true; return; end
    d = norm(q2 - q1);
    if d < 1e-8, free = ~isInCollision(robot, q1, obstacles); return; end
    n = max(2, ceil(d / step));
    for k = 0:n
        q_k = q1 + (k/n) * (q2 - q1);
        if isInCollision(robot, q_k, obstacles)
            free = false;  return;
        end
    end
    free = true;
end

function hit = isInCollision(robot, q, obstacles)
% Conservative sphere-based collision check (same approximation as isConfigFree).
% Returns true if any link sphere overlaps any obstacle bounding sphere.
    BODY_NAMES = {'link_1','link_2','link_3','link_4','link_5','link_6','tool0'};
    LINK_RADII = [ 0.055,   0.055,   0.050,   0.040,   0.038,   0.030,   0.020];
    MARGIN     = 0.015;
    hit = false;
    for b = 1:numel(BODY_NAMES)
        T = getTransform(robot, q(:), BODY_NAMES{b});
        p = T(1:3,4)';
        for k = 1:numel(obstacles)
            obs_c = obstacles{k}.T_world(1:3,4)';
            obs_r = obstacleRadius(obstacles{k});
            if norm(p - obs_c) < (LINK_RADII(b) + obs_r + MARGIN)
                hit = true;  return;
            end
        end
    end
end
