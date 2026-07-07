%% ===============================================================
%% IRB 1200 Open-Source Collision Avoidance Simulation
%% Author      : Fernando Aquilino Gatell Valor
%% Institution : Universidad Francisco de Vitoria
%%               Grado en Ingeniería en Sistemas Industriales
%% Tutor       : Roque Antonio Peña Pidal
%% Academic yr : 2025/26
%% MATLAB       : R2025b | Robotics System Toolbox
%% ===============================================================
%%
%% Description
%%   Master script that runs the full simulation demo end-to-end:
%%   1. Environment setup (paths, toolbox check, URDF verification)
%%   2. Robot model loading and kinematics validation
%%   3. Quick single-run planner comparison on Scenario 2
%%   4. Live SSM simulation with RRT* + stationary operator blocking the path
%%   5. Result plots (SSM log, joint path)
%%   6. Full 30-iter benchmark — set run_full_benchmark = true to run
%%      (~10-30 min on a modern workstation)
%%
%% Usage
%%   cd irb1200_simulation
%%   main_simulation
%% ===============================================================
clear;  clc;  close all;

%% --- Configuration ------------------------------------------
run_full_benchmark = false;   % set true to run ~30-iter benchmark (~10-30 min)

%% --- STEP 1: Environment setup ------------------------------
setup_environment();

%% --- STEP 2: Load and validate robot model ------------------
fprintf('\n[1/5] Loading ABB IRB 1200 5/90 from URDF...\n');
robot = load_irb1200();
validate_kinematics(robot);

%% --- STEP 3: Quick planner comparison - Scenario 2 ----------
fprintf('\n[2/5] Planning on Scenario 2 (central box obstacle)...\n');
fprintf('      Start: [-90o, 70o, 0o, 0o, 0o, 0o]\n');
fprintf('      Goal : [90o, 70o, 0o, 0o, 0o, 0o]\n');

obstacles = build_scenario(2);
q_start = deg2rad([-90; 70; 0; 0; 0; 0]);
q_goal  = deg2rad([90;  70; 0; 0; 0; 0]);

planners_list = {'rrt', 'rrt_connect', 'rrt_star', 'apf'};
paths_store   = cell(1, numel(planners_list));

for k = 1:numel(planners_list)
    alg = planners_list{k};
    switch alg
        case 'rrt'
            [p, i] = plan_rrt(robot, q_start, q_goal, obstacles, struct());
        case 'rrt_connect'
            [p, i] = plan_rrt_connect(robot, q_start, q_goal, obstacles, struct());
        case 'rrt_star'
            [p, i] = plan_rrt_star(robot, q_start, q_goal, obstacles, struct());
        case 'apf'
            [p, i] = plan_apf(robot, q_start, q_goal, obstacles, struct());
    end
    paths_store{k} = p;
    fprintf('  %-14s  success=%-5s  time=%6.3f s  length=%s rad\n', alg, mat2str(i.success), i.computation_time, ternary(i.success, sprintf('%.3f', i.path_length), 'N/A'));
end

% Warn if mean path length is very short (obstacles may not block)
total_length = sum(cellfun(@(p) sum(sqrt(sum(diff(p).^2, 2))), paths_store));
if total_length / numel(planners_list) < 0.3
    fprintf('[WARN] Average path length is very short (%.3f rad/planner). Obstacles may not be blocking the path.\n', total_length / numel(planners_list));
end

%% --- STEP 4: Live SSM simulation with RRT* on free space ----
fprintf('\n[3/5] Running RRT* + SSM monitoring (operator: static_block)...\n');
obs_ssm = build_scenario(1);   % free space — path guaranteed
[rrt_star_path, rrt_star_info] = plan_rrt_star(robot, q_start, q_goal, obs_ssm, struct());

this_dir = fileparts(mfilename('fullpath'));
if ~isfolder('results'), mkdir('results'); end
ssm_opts = struct('animate',    true, 'dt_nominal', 0.08, 'save_video', true, 'video_path', fullfile(this_dir, 'results', 'ssm_simulation.mp4'));

if rrt_star_info.success
    fprintf('  RRT* planned in %.3f s (%d waypoints). Starting simulation...\n', rrt_star_info.computation_time, size(rrt_star_path, 1));
    sim_log = execute_with_ssm(robot, rrt_star_path, q_goal, obs_ssm, 'static_block', ssm_opts);
else
    fprintf('  [WARN] RRT* failed on Scenario 1 (unexpected). Using stub log.\n');
    sim_log = struct('time', 0, 'distance', 1.5, 'speed_factor', 1, 'ssm_state', {{'NORMAL'}}, 'replan_times', [], 'q', zeros(1,6), 'op_pos', [1.5, 0, 0]);
end

%% --- STEP 5: Plots ------------------------------------------
%%fprintf('\n[4/5] Generating result plots...\n');

% SSM distance and speed-factor time series
%%draw_ssm_log(sim_log);

% Joint trajectory of the RRT* path
%%if ~isempty(rrt_star_path) && size(rrt_star_path,1) > 1
%%    draw_path(rrt_star_path, 'RRT*');
%%end

%% --- STEP 6: Full benchmark (optional - ~10-30 min) ---------
%%fprintf('\n[5/5] ');
%%if run_full_benchmark
%%    fprintf('Running full benchmark (~30 iter x 4 scenarios x 4 algorithms)...\n');
%%    fprintf('      Estimated time: ~%d min on a modern workstation.\n', round(4 * 4 * 30 * 2.5 / 60));
%%    bench_opts    = struct('n_iter', 30, 'verbose', true);
%%    bench_results = run_benchmark(robot, bench_opts);
%%    stats         = aggregate_stats(bench_results);
%%    save_results(bench_results, stats);
%%    draw_benchmark(stats);
%%else
%%    fprintf('Benchmark skipped (run_full_benchmark = false).\n');
%%end
%%
%%fprintf('\n[OK] Simulation complete. Results in: %s\n', fullfile(this_dir, 'results'));

%% --- Local helper -------------------------------------------
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
