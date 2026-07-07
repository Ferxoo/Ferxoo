function [path, info] = plan_apf(robot, q_start, q_goal, obstacles, opts)

narginchk(4, 5);

if nargin < 5 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'k_att',     1.0,   ...  
    'k_rep',     0.8,   ...  
    'k_rep_m',   0.0,   ...  
    'd_rep_m',   0.50,  ...  
    'step_rad',  0.008, ...  
    'max_iter',  8000,  ...  
    'goal_tol',  0.08,  ... 
    'stuck_win', 80));       

q_start = q_start(:);  q_goal = q_goal(:);
assert(numel(q_start) == 6, 'plan_apf: q_start must be 6×1. Got %d.', numel(q_start));
assert(numel(q_goal)  == 6, 'plan_apf: q_goal must be 6×1. Got %d.',  numel(q_goal));

limits = get_joint_limits();   % 6×2 [lower, upper] in radians

q_current = q_start;
path      = q_start'; 
success   = false;
stuck_cnt = 0;

min_dist_goal   = norm(q_start - q_goal);
no_progress_cnt = 0;
ESCAPE_WIN      = 300;
ESCAPE_MAG      = 0.20; 

t_start = tic;

for iter = 1:opts.max_iter

    q_err = q_goal - q_current;
    F_att = opts.k_att * q_err;

    CHECK_LINKS  = {'link_3', 'link_4', 'link_5', 'tool0'};
    LINK_RADII_R = [0.050,    0.040,    0.038,    0.020];

    F_rep = zeros(6, 1);
    for li = 1:numel(CHECK_LINKS)
        T_link = getTransform(robot, q_current, CHECK_LINKS{li});
        p_link = T_link(1:3, 4);                
        J_link = geometricJacobian(robot, q_current, CHECK_LINKS{li});
        J_lpos = J_link(4:6, :);                    

        for k = 1:numel(obstacles)
            obs   = obstacles{k};
            T_obs = obs.T_world;
            p_loc      = T_obs(1:3,1:3)' * (p_link - T_obs(1:3,4));
            p_surf_loc = nearestObsSurface(p_loc, obs.geometry);
            p_surf_w   = T_obs(1:3,1:3) * p_surf_loc + T_obs(1:3,4);
            vec_away = p_link - p_surf_w;               
            d_surf   = max(norm(vec_away) - LINK_RADII_R(li), 1e-4);

            if d_surf < opts.d_rep_m
                mag = opts.k_rep * (1/d_surf - 1/opts.d_rep_m) / (d_surf^2);
                v_n = vec_away / max(norm(vec_away), 1e-6);
                F_rep = F_rep + J_lpos' * (mag * v_n);
            end
        end
    end

    F_total = F_att + F_rep;
    f_norm  = norm(F_total);

    if f_norm < 1e-3
        stuck_cnt = stuck_cnt + 1;
        if stuck_cnt >= opts.stuck_win
            break; 
        end
    else
        stuck_cnt = 0;
    end

    q_next = q_current + opts.step_rad * F_total / max(f_norm, 1e-6);

    q_next = max(limits(:,1), min(limits(:,2), q_next));

    q_current = q_next;
    path(end+1, :) = q_current';

    if norm(q_current - q_goal) < opts.goal_tol
        success = true;
        break;
    end

    d_now = norm(q_current - q_goal);
    if d_now < min_dist_goal - 1e-4
        min_dist_goal   = d_now;
        no_progress_cnt = 0;
    else
        no_progress_cnt = no_progress_cnt + 1;
        if no_progress_cnt >= ESCAPE_WIN
            q_current       = q_current + ESCAPE_MAG * (2*rand(6,1) - 1);
            q_current       = max(limits(:,1), min(limits(:,2), q_current));
            no_progress_cnt = 0;
            stuck_cnt       = 0;
        end
    end

end

elapsed = toc(t_start);

info.success          = success;
info.computation_time = elapsed;
info.num_iterations   = iter;
if size(path, 1) >= 2
    info.path_length = sum(vecnorm(diff(path), 2, 2));
else
    info.path_length = NaN;
end

end

function opts = applyDefaults(opts, defaults)
    fields = fieldnames(defaults);
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(opts, f), opts.(f) = defaults.(f); end
    end
end

function p_surf = nearestObsSurface(p_loc, geom)
    p_loc = p_loc(:);
    if isa(geom, 'collisionBox')
        hx = geom.X / 2;  hy = geom.Y / 2;  hz = geom.Z / 2;
        hs = [hx; hy; hz];
        if any(abs(p_loc) > hs)
            p_surf = max(-hs, min(hs, p_loc));
        else
            pen     = hs - abs(p_loc);
            [~, ax] = min(pen);
            p_surf  = p_loc;
            p_surf(ax) = hs(ax) * sign_nz(p_loc(ax));
        end

    elseif isa(geom, 'collisionCylinder')
        R = geom.Radius;  H = geom.Length / 2;
        xy    = p_loc(1:2);
        r_xy  = norm(xy);
        z_c   = max(-H, min(H, p_loc(3)));
        if r_xy > 1e-8
            xy_u = xy / r_xy;
        else
            xy_u = [1; 0];
        end
        p_curved = [R * xy_u; z_c];
        p_top    = [min(r_xy, R) * xy_u; H];
        p_bot    = [min(r_xy, R) * xy_u; -H];
        d_curved = norm(p_loc - p_curved);
        d_top    = norm(p_loc - p_top);
        d_bot    = norm(p_loc - p_bot);
        [~, idx] = min([d_curved, d_top, d_bot]);
        candidates = {p_curved, p_top, p_bot};
        p_surf = candidates{idx};

    else
        r_s = 0.10;
        if isa(geom, 'collisionSphere'), r_s = geom.Radius; end
        d = norm(p_loc);
        if d > 1e-8
            p_surf = (r_s / d) * p_loc;
        else
            p_surf = [r_s; 0; 0];
        end
    end
end

function s = sign_nz(x)
    if x >= 0, s = 1; else, s = -1; end
end
