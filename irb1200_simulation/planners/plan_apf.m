%% ─────────────────────────────────────────────────────────────
% File        : plan_apf.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Artificial Potential Fields (APF) planner in joint space.
%               Attractive force pulls toward goal; repulsive force pushes
%               away from obstacle surfaces via Jacobian transpose (multi-link).
%               Included as reactive baseline for comparison.
%               Reference: Khatib, O. (1986). Real-time obstacle avoidance
%               for manipulators and mobile robots. IJRR, 5(1), 90-98.
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               q_start   — 6×1 double, start joint config [rad]
%               q_goal    — 6×1 double, goal  joint config [rad]
%               obstacles — cell array of obstacle structs (see build_scenario)
%               opts      — struct with optional fields:
%                  .k_att     (default 1.0)  attractive gain
%                  .k_rep     (default 0.8)  repulsive gain
%                  .d_rep_m   (default 0.50) obstacle influence radius [m]
%                  .step_rad  (default 0.008) gradient step [rad]
%                  .max_iter  (default 8000)
%                  .goal_tol  (default 0.08) convergence tolerance [rad]
%                  .stuck_win (default 80)   iterations to detect local min
% Outputs     : path — N×6 double, waypoints in joint space [rad]
%               info — struct: .success, .computation_time,
%                              .num_iterations, .path_length
% Dependencies: get_joint_limits, getTransform, geometricJacobian
%               (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────
% APF is susceptible to local minima (Khatib, 1986). In scenarios 3 and 4
% the success rate is expected to be significantly lower than RRT-family
% planners. This is a known limitation included for comparative purposes.

function [path, info] = plan_apf(robot, q_start, q_goal, obstacles, opts)

narginchk(4, 5);

%% Default options ─────────────────────────────────────────────
if nargin < 5 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'k_att',     1.0,   ...  % attractive field gain
    'k_rep',     0.8,   ...  % repulsive field gain
    'k_rep_m',   0.0,   ...  % unused placeholder (avoid conflict)
    'd_rep_m',   0.50,  ...  % obstacle influence distance [m]
    'step_rad',  0.008, ...  % gradient descent step [rad]
    'max_iter',  8000,  ...  % maximum iterations
    'goal_tol',  0.08,  ...  % convergence tolerance [rad] (L2 norm)
    'stuck_win', 80));       % consecutive near-zero force ticks → stuck

%% Input validation ────────────────────────────────────────────
q_start = q_start(:);  q_goal = q_goal(:);
assert(numel(q_start) == 6, 'plan_apf: q_start must be 6×1. Got %d.', numel(q_start));
assert(numel(q_goal)  == 6, 'plan_apf: q_goal must be 6×1. Got %d.',  numel(q_goal));

limits = get_joint_limits();   % 6×2 [lower, upper] in radians

%% Initialise ──────────────────────────────────────────────────
q_current = q_start;
path      = q_start';   % growing N×6 matrix (starts with start config)
success   = false;
stuck_cnt = 0;

% Progress-stall escape: if distance-to-goal hasn't improved in
% ESCAPE_WIN iterations, apply a random perturbation to break oscillations
% near convex obstacles (common when the arm must sweep a large arc in q1).
min_dist_goal   = norm(q_start - q_goal);
no_progress_cnt = 0;
ESCAPE_WIN      = 300;   % iterations without improvement before escape
ESCAPE_MAG      = 0.20;  % random perturbation magnitude [rad]

t_start = tic;

%% Gradient-descent loop ───────────────────────────────────────
for iter = 1:opts.max_iter

    %% Attractive force in joint space
    % Simple linear attraction toward the goal configuration.
    q_err = q_goal - q_current;
    F_att = opts.k_att * q_err;

    %% Repulsive force via Jacobian transpose (multi-link)
    % Four representative link origins are checked against every obstacle.
    % The nearest point on the obstacle surface (not its centre) is used to
    % compute the surface-to-link distance, providing a tighter and more
    % accurate influence region than centre-to-centre distance.
    CHECK_LINKS  = {'link_3', 'link_4', 'link_5', 'tool0'};
    LINK_RADII_R = [0.050,    0.040,    0.038,    0.020];

    F_rep = zeros(6, 1);
    for li = 1:numel(CHECK_LINKS)
        T_link = getTransform(robot, q_current, CHECK_LINKS{li});
        p_link = T_link(1:3, 4);                        % 3×1 world position
        J_link = geometricJacobian(robot, q_current, CHECK_LINKS{li});
        J_lpos = J_link(4:6, :);                        % translational rows [3×6]

        for k = 1:numel(obstacles)
            obs   = obstacles{k};
            T_obs = obs.T_world;
            % Transform link origin to obstacle local frame
            p_loc      = T_obs(1:3,1:3)' * (p_link - T_obs(1:3,4));
            % Nearest point on obstacle surface in obstacle local frame
            p_surf_loc = nearestObsSurface(p_loc, obs.geometry);
            % Convert back to world frame
            p_surf_w   = T_obs(1:3,1:3) * p_surf_loc + T_obs(1:3,4);
            % Vector from surface to link origin and surface-to-origin distance
            vec_away = p_link - p_surf_w;               % 3×1, points away from obs
            d_surf   = max(norm(vec_away) - LINK_RADII_R(li), 1e-4);

            if d_surf < opts.d_rep_m
                % Khatib (1986) repulsive potential gradient
                mag = opts.k_rep * (1/d_surf - 1/opts.d_rep_m) / (d_surf^2);
                v_n = vec_away / max(norm(vec_away), 1e-6);   % unit outward normal
                F_rep = F_rep + J_lpos' * (mag * v_n);
            end
        end
    end

    %% Total force and normalisation
    F_total = F_att + F_rep;
    f_norm  = norm(F_total);

    %% Local minimum detection
    % If the total force is negligible for stuck_win consecutive iterations,
    % the planner is likely trapped in a local minimum.
    if f_norm < 1e-3
        stuck_cnt = stuck_cnt + 1;
        if stuck_cnt >= opts.stuck_win
            break;   % declare stuck — exit without reaching goal
        end
    else
        stuck_cnt = 0;
    end

    %% Gradient step (normalised to fixed step size)
    q_next = q_current + opts.step_rad * F_total / max(f_norm, 1e-6);

    % Clamp to joint limits to prevent invalid configurations
    q_next = max(limits(:,1), min(limits(:,2), q_next));

    q_current = q_next;
    path(end+1, :) = q_current';   %#ok<AGROW>

    %% Convergence check
    if norm(q_current - q_goal) < opts.goal_tol
        success = true;
        break;
    end

    %% Progress-stall escape ───────────────────────────────────
    % If the arm hasn't reduced its joint-space distance to the goal in
    % ESCAPE_WIN iterations (oscillation near an obstacle), apply a random
    % perturbation to escape. Resets both no_progress_cnt and stuck_cnt.
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

%% Build info struct ───────────────────────────────────────────
info.success          = success;
info.computation_time = elapsed;
info.num_iterations   = iter;
if size(path, 1) >= 2
    info.path_length = sum(vecnorm(diff(path), 2, 2));
else
    info.path_length = NaN;
end

end

%% ── Local helpers ────────────────────────────────────────────

function opts = applyDefaults(opts, defaults)
    fields = fieldnames(defaults);
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(opts, f), opts.(f) = defaults.(f); end
    end
end

function p_surf = nearestObsSurface(p_loc, geom)
% Returns the nearest point on the obstacle surface to p_loc,
% expressed in the obstacle's local frame.
% For a box: clamping outside, nearest-face projection inside.
% For a cylinder: lateral surface or cap, whichever is closer.
% For a sphere / unknown: radial projection to surface.
    p_loc = p_loc(:);   % ensure 3×1
    if isa(geom, 'collisionBox')
        hx = geom.X / 2;  hy = geom.Y / 2;  hz = geom.Z / 2;
        hs = [hx; hy; hz];
        if any(abs(p_loc) > hs)
            % Outside: nearest surface point via coordinate clamping
            p_surf = max(-hs, min(hs, p_loc));
        else
            % Inside: project to the nearest face (minimum penetration depth)
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
        % Three candidate surface points: curved surface, top cap, bottom cap
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
        % Sphere or unknown geometry: radial projection to surface
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
% Sign of x, returning +1 when x == 0 (no-zero variant).
    if x >= 0, s = 1; else, s = -1; end
end
