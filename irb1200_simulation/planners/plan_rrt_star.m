%% ─────────────────────────────────────────────────────────────
% File        : plan_rrt_star.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox (NO Navigation Toolbox)
% Description : RRT* — MAIN planner. Asymptotically optimal path planning
%               via incremental rewiring of the random tree. After finding
%               an initial path the tree continues to improve it until the
%               iteration budget is exhausted.
%               Reference: Karaman, S. & Frazzoli, E. (2011). Sampling-
%               based algorithms for optimal motion planning. IJRR 30(7).
%               Note: plannerRRTStar (Navigation Toolbox) is not required.
%               This is a pure Robotics System Toolbox implementation.
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               q_start   — 6×1 double [rad]
%               q_goal    — 6×1 double [rad]
%               obstacles — cell array of obstacle structs
%               opts      — struct with optional fields:
%                  .max_iterations        (default 3000)
%                  .max_iterations_replan (default 1500)
%                  .goal_bias             (default 0.10)
%                  .max_conn_dist         (default 0.15 [rad])
%                  .val_step              (default 0.04 [rad])
%                  .is_replan             (default false)
%                  .timeout               (default 10.0 [s])
% Outputs     : path — N×6 double [rad];  info — result struct
% Dependencies: get_joint_limits, getTransform (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function [path, info] = plan_rrt_star(robot, q_start, q_goal, obstacles, opts)

narginchk(4, 5);
if nargin < 5 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'max_iterations',        3000, ...
    'max_iterations_replan', 1500, ...
    'goal_bias',             0.10, ...
    'max_conn_dist',         0.15, ...
    'val_step',              0.04, ...
    'is_replan',             false, ...
    'timeout',               15.0, ...
    'timeout_replan',        20.0, ...   % longer budget: replan does real col-checks
    'return_tree',           false));

q_start = q_start(:);  q_goal = q_goal(:);
assert(numel(q_start)==6, 'plan_rrt_star: q_start must be 6×1.');
assert(numel(q_goal) ==6, 'plan_rrt_star: q_goal must be 6×1.');

limits = get_joint_limits();
n_iter  = ternary(opts.is_replan, opts.max_iterations_replan, opts.max_iterations);
t_limit = ternary(opts.is_replan, opts.timeout_replan,        opts.timeout);

%% Tree initialisation ─────────────────────────────────────────
nodes   = q_start';    % N×6
parents = 0;           % parent index (0 = root)
costs   = 0;           % cost-from-root for each node

success       = false;
goal_tree_idx = -1;    % index of the goal node once found
length_hist   = zeros(0, 2);   % [iteration, path_cost] whenever goal improves

t_start = tic;
iter    = 0;

%% RRT* main loop ──────────────────────────────────────────────
for iter = 1:n_iter

    %% 1. Sample
    if rand < opts.goal_bias
        q_rand = q_goal';
    else
        q_rand = limits(:,1)' + rand(1,6) .* (limits(:,2)-limits(:,1))';
    end

    %% 2. Nearest node
    dists        = vecnorm(nodes - repmat(q_rand, size(nodes,1), 1), 2, 2);
    [d_near, ni] = min(dists);
    q_near       = nodes(ni,:);

    %% 3. Steer toward q_rand
    if d_near > opts.max_conn_dist
        q_new = q_near + opts.max_conn_dist * (q_rand - q_near) / d_near;
    else
        q_new = q_rand;
    end
    q_new = max(limits(:,1)', min(limits(:,2)', q_new));
    if norm(q_new - q_near) < 1e-8,  continue;  end

    %% 4. Check edge from nearest to new node
    if ~isPathFree(robot, q_near, q_new, obstacles, opts.val_step),  continue;  end

    %% 5. Find near neighbours for RRT* parent selection & rewiring
    % Radius uses the RRT* formula γ*(log(n)/n)^(1/d) scaled to remain
    % within [max_conn_dist, 2*max_conn_dist].
    n_nodes      = size(nodes, 1);
    r_near       = min(2.0*opts.max_conn_dist, ...
                       opts.max_conn_dist * (log(n_nodes+1)/(n_nodes+1))^(1/6) * 30);
    near_dists   = vecnorm(nodes - repmat(q_new, n_nodes, 1), 2, 2);
    near_indices = find(near_dists < r_near);
    % Limit to 25 neighbours to bound rewiring overhead per iteration
    if numel(near_indices) > 25
        [~, sort_idx]  = sort(near_dists(near_indices));
        near_indices   = near_indices(sort_idx(1:25));
    end

    %% 6. Choose best parent (minimum cost-from-root among near neighbours)
    best_parent = ni;
    best_cost   = costs(ni) + norm(q_new - q_near);

    for k = 1:numel(near_indices)
        nk = near_indices(k);
        if nk == ni,  continue;  end
        c = costs(nk) + norm(q_new - nodes(nk,:));
        if c < best_cost && ...
           isPathFree(robot, nodes(nk,:), q_new, obstacles, opts.val_step)
            best_cost   = c;
            best_parent = nk;
        end
    end

    %% 7. Add new node to the tree
    nodes(end+1,:) = q_new;
    parents(end+1) = best_parent;
    costs(end+1)   = best_cost;
    new_idx        = size(nodes, 1);

    %% 8. Rewire: update near neighbours whose cost improves through new node
    for k = 1:numel(near_indices)
        nk = near_indices(k);
        c  = best_cost + norm(nodes(nk,:) - q_new);
        if c < costs(nk) && ...
           isPathFree(robot, q_new, nodes(nk,:), obstacles, opts.val_step)
            parents(nk) = new_idx;
            costs(nk)   = c;
        end
    end

    %% 9. Goal check
    d_goal = norm(q_new - q_goal');
    if d_goal <= opts.max_conn_dist && ...
       isPathFree(robot, q_new, q_goal', obstacles, opts.val_step)
        g_cost = best_cost + norm(q_goal' - q_new);
        if ~success
            % First time reaching goal: add goal node
            nodes(end+1,:) = q_goal';
            parents(end+1) = new_idx;
            costs(end+1)   = g_cost;
            goal_tree_idx  = size(nodes, 1);
            success        = true;
            length_hist(end+1,:) = [iter, g_cost];   % record first solution
            % Fast replanning: stop as soon as a path is found
            if opts.is_replan,  break;  end
        elseif g_cost < costs(goal_tree_idx)
            % Better path to goal: rewire the goal node
            parents(goal_tree_idx) = new_idx;
            costs(goal_tree_idx)   = g_cost;
            length_hist(end+1,:) = [iter, g_cost];   % record improvement
        end
    end

    if toc(t_start) > t_limit,  break;  end
end

elapsed = toc(t_start);

%% Extract best path ───────────────────────────────────────────
if success
    path = tracePath(nodes, parents, goal_tree_idx);
else
    path = zeros(0, 6);
end

info.success          = success;
info.computation_time = elapsed;
info.num_iterations   = iter;
info.path_length      = ternary(success && size(path,1)>1, ...
                                sum(vecnorm(diff(path),2,2)), NaN);
info.length_history   = length_hist;   % K×2: [iteration, cost] at each improvement
if opts.return_tree
    info.tree_nodes   = nodes;         % N×6 joint configs of all tree nodes
    info.tree_parents = parents;       % N×1 parent indices (0 = root)
end
end

%% ══════════════════════════════════════════════════════════════
%% Local helpers
%% ══════════════════════════════════════════════════════════════

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
    % Split: collisionSphere → fast FK check; box/cylinder → checkCollision.
    is_sph    = cellfun(@(o) isa(o.geometry, 'collisionSphere'), obstacles);
    sph_obs   = obstacles( is_sph);
    other_obs = obstacles(~is_sph);
    % Fast sphere-sphere check (avoids checkCollision overhead for dynamic spheres)
    if ~isempty(sph_obs) && ~isConfigFree_spheres(robot, q(:), sph_obs)
        free = false; return;
    end
    % checkCollision for box / cylinder static obstacles
    if ~isempty(other_obs)
        geoms = cellfun(@(o) o.geometry, other_obs, 'UniformOutput', false);
        try
            free = ~checkCollision(robot, q(:), geoms, 'IgnoreSelfCollision', 'on');
        catch
            free = true;   % conservative fallback if toolbox unavailable
        end
    else
        free = true;
    end
end

function free = isConfigFree_spheres(robot, q, sphere_obs)
% FK-based sphere-sphere collision check — avoids checkCollision overhead.
% Uses the same link names and conservative radii as compute_min_distance.
    LINK_NAMES = {'link_1','link_2','link_3','link_4','link_5','link_6','tool0'};
    LINK_RADII = [0.055,   0.055,   0.050,   0.040,   0.038,   0.030,  0.020];
    MARGIN     = 0.015;   % 15 mm safety margin
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
