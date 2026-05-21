%% ─────────────────────────────────────────────────────────────
% File        : run_benchmark.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Runs the comparative benchmark: N iterations × 4 scenarios
%               × 4 algorithms. Saves intermediate results after each
%               algorithm-scenario block.
% Inputs      : robot — rigidBodyTree (DataFormat='column')
%               opts  — struct with optional fields:
%                  .n_iter             (default 30)
%                  .scenarios          (default [1,2,3,4])
%                  .algorithms         (default {'rrt','rrt_connect','rrt_star','apf'})
%                  .verbose            (default true)
%                  .save_intermediate  (default true)
% Outputs     : results — struct array (n_scenarios × n_algorithms) with
%                  .algorithm, .scenario, .metrics(i) per iteration
% Dependencies: build_scenario, plan_rrt, plan_rrt_connect, plan_rrt_star,
%               plan_apf, compute_metrics, save (MATLAB built-in)
%% ─────────────────────────────────────────────────────────────

function results = run_benchmark(robot, opts)

narginchk(1, 2);

%% Default options ─────────────────────────────────────────────
if nargin < 2 || isempty(opts), opts = struct(); end
opts = applyDefaults(opts, struct(...
    'n_iter',            30, ...
    'scenarios',         [1, 2, 3, 4], ...
    'algorithms',        {{'rrt','rrt_connect','rrt_star','apf'}}, ...
    'verbose',           true, ...
    'save_intermediate', true));

%% Fixed start/goal configurations — shared across all scenarios ─
% q_start: base at -90° (robot pointing right), q2=70° (arm raised)
% q_goal:  base at +90° (robot pointing left),  q2=70° (arm raised)
% The arc sweeps 180° in q1 and forces the arm to cross the centre of the
% workspace, ensuring that the obstacles in Scenarios 2–4 block the path.
Q_START = deg2rad([-90; 70; 0; 0; 0; 0]);
Q_GOAL  = deg2rad([ 90; 70; 0; 0; 0; 0]);

CONFIGS = {
    {Q_START, Q_GOAL};   % Scenario 1 — free space
    {Q_START, Q_GOAL};   % Scenario 2 — central box
    {Q_START, Q_GOAL};   % Scenario 3 — corridor
    {Q_START, Q_GOAL};   % Scenario 4 — dense obstacles
};

%% Global random seed for reproducibility ─────────────────────
rng(42);

n_sc  = numel(opts.scenarios);
n_alg = numel(opts.algorithms);

%% Main benchmark loops ────────────────────────────────────────
for s_idx = 1:n_sc
    s         = opts.scenarios(s_idx);
    obstacles = build_scenario(s);
    q_start   = CONFIGS{s}{1};
    q_goal    = CONFIGS{s}{2};

    if opts.verbose
        fprintf('\n══ Scenario %d (%d obstacles) ══\n', s, numel(obstacles));
    end

    for a_idx = 1:n_alg
        alg = opts.algorithms{a_idx};
        results(s_idx, a_idx).algorithm = alg;  %#ok<AGROW>
        results(s_idx, a_idx).scenario  = s;

        if opts.verbose
            fprintf('  Algorithm: %-12s\n', alg);
        end

        for i = 1:opts.n_iter

            % Run the planner
            switch alg
                case 'rrt'
                    [path, info] = plan_rrt(robot, q_start, q_goal, obstacles, struct());
                case 'rrt_connect'
                    [path, info] = plan_rrt_connect(robot, q_start, q_goal, obstacles, struct());
                case 'rrt_star'
                    [path, info] = plan_rrt_star(robot, q_start, q_goal, obstacles, struct());
                case 'apf'
                    [path, info] = plan_apf(robot, q_start, q_goal, obstacles, struct());
                otherwise
                    error('run_benchmark: unknown algorithm "%s".', alg);
            end

            metrics = compute_metrics(path, info.computation_time, info.success);
            results(s_idx, a_idx).metrics(i) = metrics;

            if opts.verbose
                len_str = num2str(ternary(info.success, info.path_length, NaN), '%6.3f');
                fprintf('    S%d | %-12s | %2d/%d | t=%6.3f s | len=%s | %s\n', ...
                        s, alg, i, opts.n_iter, info.computation_time, len_str, ...
                        ternary(info.success, 'OK', 'FAIL'));
            end

        end % iterations

        % Save intermediate results after each algorithm-scenario block
        % so data is not lost if a long benchmark is interrupted.
        if opts.save_intermediate
            results_path = fullfile('results', 'benchmark_raw.mat');
            if ~isfolder('results'), mkdir('results'); end
            save(results_path, 'results');
        end

    end % algorithms
end % scenarios

if opts.verbose
    fprintf('\n[run_benchmark] Complete: %d scenarios × %d algorithms × %d iterations.\n', ...
            n_sc, n_alg, opts.n_iter);
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

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
