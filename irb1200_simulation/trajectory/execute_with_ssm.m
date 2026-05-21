%% ─────────────────────────────────────────────────────────────
% File        : execute_with_ssm.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Animates a joint-space trajectory while continuously
%               monitoring the robot-operator distance via SSM. Slows
%               down or stops and replans when ISO/TS 15066 thresholds
%               are breached.
% Inputs      : robot         — rigidBodyTree (DataFormat='column')
%               path          — N×6 double, planned joint waypoints [rad]
%               q_goal        — 6×1 double, goal joint config [rad]
%               obstacles     — cell array of obstacle structs
%               operator_mode — char: 'static'|'approach'|'lateral'|
%                              'random'|'slow_then_stop'
%               opts          — struct with optional fields:
%                  .animate      (default true)
%                  .dt_nominal   (default 0.08 [s])
%                  .v_max        (default 2.0 [rad/s])
%                  .plot_live    (default true)
%                  .save_video   (default false) — record MPEG-4 video
%                  .video_path   (default 'results/ssm_simulation.mp4')
% Outputs     : log — struct with fields:
%                  .time, .q, .op_pos, .distance, .ssm_state,
%                  .speed_factor, .replan_times
% Dependencies: get_operator_position, compute_min_distance,
%               ssm_state_machine, plan_rrt_star, draw_scene
%% ─────────────────────────────────────────────────────────────

function log = execute_with_ssm(robot, path, q_goal, obstacles, operator_mode, opts)

narginchk(5, 6);

%% Default options ─────────────────────────────────────────────
if nargin < 6 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'animate',    true,  ...
    'dt_nominal', 0.08,  ...   % [s] nominal time step at full speed
    'v_max',      2.0,   ...   % [rad/s] conservative joint velocity cap
    'plot_live',  true,  ...
    'save_video', false, ...   % record MPEG-4 video of the animation
    'video_path', 'results/ssm_simulation.mp4'));  % output video path

%% Input validation ────────────────────────────────────────────
assert(size(path, 2) == 6 && size(path, 1) >= 2, ...
    'execute_with_ssm: path must be N×6 with N≥2.');
q_goal = q_goal(:);

%% Interpolate planner waypoints to a smooth trajectory ────────
% The raw planner output has uneven spacing. We remap to uniform
% time steps using PCHIP spline interpolation to avoid velocity
% discontinuities while preserving the path shape.
n_path = size(path, 1);
t_path = linspace(0, n_path * 0.5, n_path);   % ~0.5 s per waypoint
t_fine = 0 : opts.dt_nominal : t_path(end);
q_traj = interp1(t_path, path, t_fine, 'pchip');   % (M×6) smooth trajectory

%% Execution state ─────────────────────────────────────────────
ssm_state      = 'NORMAL';
t              = 0;
idx            = 1;
replan_cnt     = 0;
stop_ticks     = 0;          % consecutive ticks spent in STOP_REPLAN
MAX_STOP_TICKS = 300;        % ~12 s at dt=0.08; break if operator never leaves

% Pre-allocate log arrays (will grow dynamically — small overhead acceptable
% for simulation durations < 5 min)
log.time         = [];
log.q            = zeros(0, 6);
log.op_pos       = zeros(0, 3);
log.distance     = [];
log.ssm_state    = {};
log.speed_factor = [];
log.replan_times = [];
log.d_min_post   = [];   % min dist to operator along each replanned traj (§5.5)

if opts.animate
    fig = figure('Name', 'IRB 1200 SSM Simulation', 'NumberTitle', 'off', ...
                 'Position', [50, 50, 900, 650]);
end

% [Change 1.3] Open VideoWriter when save_video=true and animation is active
vw = [];
if opts.save_video && opts.animate
    vw           = VideoWriter(opts.video_path, 'MPEG-4');
    vw.FrameRate = 25;   % 25 fps; renders every 3 ticks → ~6× real-time playback
    open(vw);
    fprintf('[execute_with_ssm] Recording video: %s\n', opts.video_path);
end

%% Main execution loop ─────────────────────────────────────────
while idx <= size(q_traj, 1)

    q_now  = q_traj(idx, :)';
    op_pos = get_operator_position(t, operator_mode);

    % --- SSM monitoring ----------------------------------------
    [dist, ~] = compute_min_distance(robot, q_now, op_pos);
    [ssm_state, sf, do_replan] = ssm_state_machine(dist, ssm_state);

    % --- Log current state -------------------------------------
    log.time(end+1)         = t;
    log.q(end+1, :)         = q_now';
    log.op_pos(end+1, :)    = op_pos;
    log.distance(end+1)     = dist;
    log.ssm_state{end+1}    = ssm_state;
    log.speed_factor(end+1) = sf;

    % --- Live animation (every 3rd step to maintain real-time rate)
    if opts.animate && mod(idx, 3) == 0
        figure(fig);
        cla;
        draw_scene(robot, q_now, obstacles, op_pos, ssm_state);
        title(sprintf('t = %.2f s | State: %-12s | dist = %.3f m | sf = %.2f', ...
                      t, ssm_state, dist, sf), 'FontSize', 10);
        if ~isempty(vw)
            drawnow;                        % [Change 1.3] full render for capture
            writeVideo(vw, getframe(fig));  % [Change 1.3] write frame to video
        else
            drawnow limitrate;
        end
    end

    % --- Handle STOP_REPLAN transition -------------------------
    if do_replan
        replan_cnt             = replan_cnt + 1;
        log.replan_times(end+1) = t;
        fprintf('[SSM] STOP_REPLAN at t=%.2f s — replanning...\n', t);

        replan_opts            = struct();
        replan_opts.is_replan  = true;
        replan_opts.max_iterations_replan = 8000;
        replan_opts.timeout_replan        = 30.0;

        % [REPLAN FIX] Add operator as temporary sphere obstacle so RRT* routes around them
        op_obs.name          = 'operator';
        op_obs.geometry      = collisionSphere(0.50);   % 700 mm: guarantees replanned path stays in SLOW zone (d_surf_SSM ≥ 0.565 m > D_STOP=0.50 m)
        op_obs.T_world       = trvec2tform(op_pos(:)');
        op_obs.geometry.Pose = op_obs.T_world;
        op_obs.color         = [0.20, 0.60, 1.00];
        obstacles_with_op    = [obstacles, {op_obs}];   % static scene + operator

        [new_path, rinfo] = plan_rrt_star(robot, q_now, q_goal, obstacles_with_op, replan_opts);

        if rinfo.success
            fprintf('[SSM] Replan OK in %.3f s (%d iter). Resuming.\n', ...
                    rinfo.computation_time, rinfo.num_iterations);
            % Compute min distance to operator along replanned path (§5.5)
            % Must be > 0.50 m before execution to confirm safety
            op_pos_now = get_operator_position(t, operator_mode);
            d_post = inf;
            for wp = 1:size(new_path,1)
                [d_wp, ~] = compute_min_distance(robot, new_path(wp,:)', op_pos_now);
                if d_wp < d_post, d_post = d_wp; end
            end
            log.d_min_post(end+1) = d_post;
            fprintf('[SSM] d_min_post = %.3f m (must be > 0.50 m).\n', d_post);
            % Rebuild smooth trajectory from the replanned path
            n_new   = size(new_path, 1);
            t_new   = linspace(0, n_new * 0.5, n_new);
            t_fine2 = 0 : opts.dt_nominal : t_new(end);
            q_traj  = interp1(t_new, new_path, t_fine2, 'pchip');
            idx     = 1;
            continue;
        else
            fprintf('[SSM] Replan FAILED after %.3f s. Robot holds position.\n', ...
                    rinfo.computation_time);
            break;
        end
    end

    % --- Advance trajectory index based on speed factor --------
    % sf = 1 → advance at full speed (2 steps per dt to get slightly faster
    % playback); sf = 0 in STOP_REPLAN → no advance.
    if strcmp(ssm_state, 'STOP_REPLAN')
        advance     = 0;
        stop_ticks  = stop_ticks + 1;
        % Safety guard: if the operator never leaves the stop zone (e.g.,
        % 'approach' mode keeps them inside) the loop would run forever.
        % After MAX_STOP_TICKS we log the situation and break.
        if stop_ticks >= MAX_STOP_TICKS
            fprintf('[SSM] Stop zone maintained for %.1f s. Terminating.\n', ...
                    stop_ticks * opts.dt_nominal);
            break;
        end
    else
        advance    = max(1, round(sf * 2));
        stop_ticks = 0;   % reset counter whenever we leave STOP_REPLAN
    end
    idx = idx + advance;
    t   = t + opts.dt_nominal;

end

% [Change 1.3] Close video writer if recording was active
if ~isempty(vw)
    close(vw);
    fprintf('[execute_with_ssm] Video saved: %s\n', opts.video_path);
end

fprintf('[execute_with_ssm] Finished: %d log entries, %d replan(s).\n', ...
        numel(log.time), replan_cnt);

end

%% ── Local helper ─────────────────────────────────────────────

function opts = applyDefaults(opts, defaults)
    fields = fieldnames(defaults);
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(opts, f), opts.(f) = defaults.(f); end
    end
end
