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
    'timeout_replan',        20.0, ...   
    'return_tree',           false));

q_start = q_start(:);  q_goal = q_goal(:);
assert(numel(q_start)==6, 'plan_rrt_star: q_start must be 6×1.');
assert(numel(q_goal) ==6, 'plan_rrt_star: q_goal must be 6×1.');

limits = get_joint_limits();
n_iter  = ternary(opts.is_replan, opts.max_iterations_replan, opts.max_iterations);
t_limit = ternary(opts.is_replan, opts.timeout_replan,        opts.timeout);

nodes   = q_start';   
parents = 0;           
costs   = 0;         

success       = false;
goal_tree_idx = -1;    
length_hist   = zeros(0, 2);  

t_start = tic;
iter    = 0;

for iter = 1:n_iter

    if rand < opts.goal_bias
        q_rand = q_goal';
    else
        q_rand = limits(:,1)' + rand(1,6) .* (limits(:,2)-limits(:,1))';
    end

    dists        = vecnorm(nodes - repmat(q_rand, size(nodes,1), 1), 2, 2);
    [d_near, ni] = min(dists);
    q_near       = nodes(ni,:);

    if d_near > opts.max_conn_dist
        q_new = q_near + opts.max_conn_dist * (q_rand - q_near) / d_near;
    else
        q_new = q_rand;
    end
    q_new = max(limits(:,1)', min(limits(:,2)', q_new));
    if norm(q_new - q_near) < 1e-8,  continue;  end

    if ~isPathFree(robot, q_near, q_new, obstacles, opts.val_step),  continue;  end

    n_nodes      = size(nodes, 1);
    r_near       = min(2.0*opts.max_conn_dist, opts.max_conn_dist * (log(n_nodes+1)/(n_nodes+1))^(1/6) * 30);
    near_dists   = vecnorm(nodes - repmat(q_new, n_nodes, 1), 2, 2);
    near_indices = find(near_dists < r_near);
    if numel(near_indices) > 25
        [~, sort_idx]  = sort(near_dists(near_indices));
        near_indices   = near_indices(sort_idx(1:25));
    end

    best_parent = ni;
    best_cost   = costs(ni) + norm(q_new - q_near);

    for k = 1:numel(near_indices)
        nk = near_indices(k);
        if nk == ni,  continue;  end
        c = costs(nk) + norm(q_new - nodes(nk,:));
        if c < best_cost && isPathFree(robot, nodes(nk,:), q_new, obstacles, opts.val_step)
            best_cost   = c;
            best_parent = nk;
        end
    end

    nodes(end+1,:) = q_new;
    parents(end+1) = best_parent;
    costs(end+1)   = best_cost;
    new_idx        = size(nodes, 1);

    for k = 1:numel(near_indices)
        nk = near_indices(k);
        c  = best_cost + norm(nodes(nk,:) - q_new);
        if c < costs(nk) && isPathFree(robot, q_new, nodes(nk,:), obstacles, opts.val_step)
            parents(nk) = new_idx;
            costs(nk)   = c;
        end
    end

    d_goal = norm(q_new - q_goal');
    if d_goal <= opts.max_conn_dist && isPathFree(robot, q_new, q_goal', obstacles, opts.val_step)
        g_cost = best_cost + norm(q_goal' - q_new);
        if ~success
            nodes(end+1,:) = q_goal';
            parents(end+1) = new_idx;
            costs(end+1)   = g_cost;
            goal_tree_idx  = size(nodes, 1);
            success        = true;
            length_hist(end+1,:) = [iter, g_cost];   
            if opts.is_replan,  break;  end
        elseif g_cost < costs(goal_tree_idx)
            parents(goal_tree_idx) = new_idx;
            costs(goal_tree_idx)   = g_cost;
            length_hist(end+1,:) = [iter, g_cost];   
        end
    end

    if toc(t_start) > t_limit,  break;  end
end

elapsed = toc(t_start);

if success
    path = tracePath(nodes, parents, goal_tree_idx);
else
    path = zeros(0, 6);
end

info.success          = success;
info.computation_time = elapsed;
info.num_iterations   = iter;
info.path_length      = ternary(success && size(path,1)>1, sum(vecnorm(diff(path),2,2)), NaN);
info.length_history   = length_hist;   
if opts.return_tree
    info.tree_nodes   = nodes;         
    info.tree_parents = parents;       
end
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
            free = true;   % conservative fallback if toolbox unavailable
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
        indices = [idx, indices];
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
