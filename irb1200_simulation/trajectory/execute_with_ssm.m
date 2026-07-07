function log = execute_with_ssm(robot, path, q_goal, obstacles, operator_mode, opts)

narginchk(5, 6);

if nargin < 6 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'animate',    true,  ...
    'dt_nominal', 0.08,  ...   
    'v_max',      2.0,   ...  
    'plot_live',  true,  ...
    'save_video', false, ...   
    'video_path', 'results/ssm_simulation.mp4')); 

assert(size(path, 2) == 6 && size(path, 1) >= 2, 'execute_with_ssm: path must be N×6 with N≥2.');
q_goal = q_goal(:);

n_path = size(path, 1);
t_path = linspace(0, n_path * 0.5, n_path);   % ~0.5 s per waypoint
t_fine = 0 : opts.dt_nominal : t_path(end);
q_traj = interp1(t_path, path, t_fine, 'pchip');   % (M×6) smooth trajectory

ssm_state      = 'NORMAL';
t              = 0;
idx            = 1;
tick           = 0;                       
replan_cnt     = 0;
avoiding       = false;      
replan_pending = false;                                
AVOID_SPEED    = 0.15;      
is_static_block    = strcmpi(operator_mode, 'static_block');
APPROACH_DURATION  = 4.0;   
Z_OPERATOR         = 0.55;   
CLOSER_FACTOR      = 0.70;  
if is_static_block
    q_mid     = path(ceil(size(path, 1) / 2), :)';
    T_mid     = getTransform(robot, q_mid, 'tool0');
    block_pos = [T_mid(1,4)*CLOSER_FACTOR, T_mid(2,4)*CLOSER_FACTOR, Z_OPERATOR];
    safe_pos  = [1.5, 0.0, Z_OPERATOR];   
end

log.time         = [];
log.q            = zeros(0, 6);
log.op_pos       = zeros(0, 3);
log.distance     = [];
log.ssm_state    = {};
log.speed_factor = [];
log.replan_times = [];
log.d_min_post   = [];   

if opts.animate
    fig = figure('Name', 'IRB 1200 SSM Simulation', 'NumberTitle', 'off', 'Position', [50, 50, 1500, 700]);
    ax_main = subplot(1, 2, 1);
    ax_side = subplot(1, 2, 2);
end

vw = [];
if opts.save_video && opts.animate
    vw           = VideoWriter(opts.video_path, 'MPEG-4');
    vw.FrameRate = 25;   % 25 fps; renders every 3 ticks → ~6× real-time playback
    open(vw);
    fprintf('[execute_with_ssm] Recording video: %s\n', opts.video_path);
end

while idx <= size(q_traj, 1)

    tick = tick + 1;
    q_now  = q_traj(idx, :)';
    if is_static_block
        if t < APPROACH_DURATION
            op_pos = safe_pos + (t / APPROACH_DURATION) * (block_pos - safe_pos);
        else
            op_pos = block_pos;  
        end
    else
        op_pos = get_operator_position(t, operator_mode);
    end

    operator_settled = ~is_static_block || t >= APPROACH_DURATION;

    [dist, ~] = compute_min_distance(robot, q_now, op_pos);
    if avoiding
        [probe_state, probe_sf, probe_replan] = ssm_state_machine(dist, 'NORMAL');
        if strcmp(probe_state, 'STOP_REPLAN')
            ssm_state = 'SLOW';
            sf        = AVOID_SPEED;
            do_replan = false;
        else
            avoiding  = false;
            ssm_state = probe_state;
            sf        = probe_sf;
            do_replan = probe_replan;
        end
    else
        [ssm_state, sf, do_replan] = ssm_state_machine(dist, ssm_state);
        if strcmp(ssm_state, 'STOP_REPLAN') && ~operator_settled
            replan_pending = true;
            do_replan      = false;
        elseif ~strcmp(ssm_state, 'STOP_REPLAN')
            replan_pending = false;   % no longer needed
        end
    end
    
    log.time(end+1)         = t;
    log.q(end+1, :)         = q_now';
    log.op_pos(end+1, :)    = op_pos;
    log.distance(end+1)     = dist;
    log.ssm_state{end+1}    = ssm_state;
    log.speed_factor(end+1) = sf;

    if opts.animate && mod(tick, 3) == 0
        figure(fig);
        cla(ax_main);
        draw_scene(robot, q_now, obstacles, op_pos, ssm_state, ax_main, [45, 25]);
        title(ax_main, sprintf('t = %.2f s | State: %-12s | dist = %.3f m | sf = %.2f', t, ssm_state, dist, sf), 'FontSize', 10);

        cla(ax_side);
        draw_scene(robot, q_now, obstacles, op_pos, ssm_state, ax_side, [0, 10]);
        title(ax_side, 'Vista lateral — holgura robot-operario', 'FontSize', 10);

        if ~isempty(vw)
            drawnow;                      
            writeVideo(vw, getframe(fig));  
        else
            drawnow limitrate;
        end
    end

    if do_replan || (replan_pending && operator_settled)
        replan_pending          = false;
        replan_cnt              = replan_cnt + 1;
        log.replan_times(end+1) = t;
        fprintf('[SSM] STOP_REPLAN at t=%.2f s — replanning...\n', t);

        replan_opts            = struct();
        replan_opts.is_replan  = true;
        replan_opts.max_iterations_replan = 8000;
        replan_opts.timeout_replan        = 30.0;

        R_OP                 = 0.150;    
        MARGIN_CFG           = 0.015;    
        DESIRED_AVOID_MARGIN = 0.45;     
        r_obs = min(DESIRED_AVOID_MARGIN, dist + R_OP - MARGIN_CFG - 0.01);
        r_obs = max(R_OP, r_obs);
        op_obs.name          = 'operator';
        op_obs.geometry      = collisionSphere(r_obs);
        op_obs.T_world       = trvec2tform(op_pos(:)');
        op_obs.geometry.Pose = op_obs.T_world;
        op_obs.color         = [0.20, 0.60, 1.00];
        obstacles_with_op    = [obstacles, {op_obs}];  

        [new_path, rinfo] = plan_rrt_star(robot, q_now, q_goal, obstacles_with_op, replan_opts);
        t = t + rinfo.computation_time;   

        if rinfo.success
            old_remaining_path = q_traj(idx:end, :);

            n_before  = size(new_path, 1);
            new_path  = smooth_path(robot, new_path, obstacles_with_op);
            fprintf('[SSM] Replan OK in %.3f s (%d iter). Smoothed %d -> %d waypoints. Creeping through the detour...\n', rinfo.computation_time, rinfo.num_iterations, n_before, size(new_path, 1));
            avoiding = true;
            if is_static_block
                op_pos_now = block_pos;   % still stationary at this point
            else
                op_pos_now = get_operator_position(t, operator_mode);
            end
            d_post = inf;
            for wp = 2:size(new_path,1)
                [d_wp, ~] = compute_min_distance(robot, new_path(wp,:)', op_pos_now);
                if d_wp < d_post, d_post = d_wp; end
            end
            log.d_min_post(end+1) = d_post;
            fprintf('[SSM] Tightest clearance to current operator position along new path: %.3f m.\n', d_post);

            if opts.animate
                fig_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results', 'figures');
                if ~isfolder(fig_dir), mkdir(fig_dir); end
                fig_path = fullfile(fig_dir, sprintf('trajectory_comparison_replan%d.png', replan_cnt));
                draw_trajectory_comparison(robot, old_remaining_path, new_path, op_pos_now, fig_path);
                fprintf('[SSM] Trajectory comparison saved: %s\n', fig_path);
            end

            n_new   = size(new_path, 1);
            t_new   = linspace(0, n_new * 0.5, n_new);
            t_fine2 = 0 : opts.dt_nominal : t_new(end);
            q_traj  = interp1(t_new, new_path, t_fine2, 'pchip');
            idx     = 1;
            continue;
        else
            fprintf('[SSM] Replan FAILED after %.3f s. Robot holds position.\n', rinfo.computation_time);
            break;
        end
    end

    if strcmp(ssm_state, 'STOP_REPLAN')
        advance = 0;
    else
        advance = max(1, round(sf * 2));
    end
    idx = idx + advance;
    t   = t + opts.dt_nominal;

end

if is_static_block && idx > size(q_traj, 1)
    fprintf('[SSM] Goal reached — operator retreating.\n');
    q_final    = q_traj(end, :)';
    N_RETREAT  = 80;   
    for kk = 1:N_RETREAT
        frac   = kk / N_RETREAT;
        op_pos = block_pos + frac * (safe_pos - block_pos);
        [dist, ~] = compute_min_distance(robot, q_final, op_pos);

        log.time(end+1)         = t;
        log.q(end+1, :)         = q_final';
        log.op_pos(end+1, :)    = op_pos;
        log.distance(end+1)     = dist;
        log.ssm_state{end+1}    = 'NORMAL';
        log.speed_factor(end+1) = 0;

        if opts.animate && mod(kk, 3) == 0
            figure(fig);
            cla(ax_main);
            draw_scene(robot, q_final, obstacles, op_pos, 'NORMAL', ax_main, [45, 25]);
            title(ax_main, sprintf('t = %.2f s | GOAL REACHED — operator retreating | dist = %.3f m', t, dist), 'FontSize', 10);

            cla(ax_side);
            draw_scene(robot, q_final, obstacles, op_pos, 'NORMAL', ax_side, [0, 10]);
            title(ax_side, 'Vista lateral — holgura robot-operario', 'FontSize', 10);

            if ~isempty(vw)
                drawnow;
                writeVideo(vw, getframe(fig));
            else
                drawnow limitrate;
            end
        end
        t = t + opts.dt_nominal;
    end
end

if ~isempty(vw)
    close(vw);
    fprintf('[execute_with_ssm] Video saved: %s\n', opts.video_path);
end

fprintf('[execute_with_ssm] Finished: %d log entries, %d replan(s).\n', numel(log.time), replan_cnt);

end

function opts = applyDefaults(opts, defaults)
    fields = fieldnames(defaults);
    for k = 1:numel(fields)
        f = fields{k};
        if ~isfield(opts, f), opts.(f) = defaults.(f); end
    end
end
