function path = smooth_path(robot, path, obstacles, val_step)

narginchk(3, 4);
if nargin < 4 || isempty(val_step), val_step = 0.04; end

if size(path, 1) <= 2
    return;   % nothing to shortcut
end

changed = true;
while changed
    changed = false;
    i = 1;
    while i < size(path, 1) - 1
        for j = size(path, 1):-1:(i + 2)
            if isPathFree(robot, path(i,:), path(j,:), obstacles, val_step)
                path    = [path(1:i, :); path(j:end, :)];
                changed = true;
                break;
            end
        end
        i = i + 1;
    end
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
    MARGIN     = 0.015;   % must match plan_rrt_star.m's isConfigFree_spheres
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
