function save_results(results, stats)

narginchk(2, 2);

this_dir    = fileparts(mfilename('fullpath'));
results_dir = fullfile(this_dir, '..', 'results');
if ~isfolder(results_dir)
    mkdir(results_dir);
end

mat_path = fullfile(results_dir, 'benchmark_raw.mat');
save(mat_path, 'results', 'stats');
fprintf('[save_results] .mat saved: %s\n', mat_path);

[n_sc, n_alg] = size(stats);

scenario_col    = zeros(n_sc * n_alg, 1);
algorithm_col   = cell(n_sc * n_alg, 1);
success_col     = zeros(n_sc * n_alg, 1);
time_mean_col   = zeros(n_sc * n_alg, 1);
time_std_col    = zeros(n_sc * n_alg, 1);
length_mean_col = zeros(n_sc * n_alg, 1);
length_std_col  = zeros(n_sc * n_alg, 1);
jerk_mean_col   = zeros(n_sc * n_alg, 1);
jerk_std_col    = zeros(n_sc * n_alg, 1);

row = 0;
for s = 1:n_sc
    for a = 1:n_alg
        row = row + 1;
        st  = stats(s, a);
        scenario_col(row)    = st.scenario;
        algorithm_col{row}   = st.algorithm;
        success_col(row)     = st.success_rate;
        time_mean_col(row)   = st.time_mean;
        time_std_col(row)    = st.time_std;
        length_mean_col(row) = st.length_mean;
        length_std_col(row)  = st.length_std;
        jerk_mean_col(row)   = st.jerk_mean;
        jerk_std_col(row)    = st.jerk_std;
    end
end

T = table(scenario_col, algorithm_col, success_col, ...
          time_mean_col, time_std_col, ...
          length_mean_col, length_std_col, ...
          jerk_mean_col, jerk_std_col, ...
    'VariableNames', {'scenario','algorithm','success_rate_pct', ...
                      'time_mean_s','time_std_s', ...
                      'length_mean_rad','length_std_rad', ...
                      'jerk_mean_rad_s3','jerk_std_rad_s3'});

csv_path = fullfile(results_dir, 'benchmark_summary.csv');
writetable(T, csv_path);
fprintf('[save_results] CSV saved: %s\n', csv_path);

sc_names = {'Free space', 'Central box', 'Corridor', 'Dense'};

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════════════════════╗\n');
fprintf('║           BENCHMARK SUMMARY — ABB IRB 1200 | N=30 iter/combination       ║\n');
fprintf('╠══════════╦══════════════╦════════╦══════════════════╦═══════════════════╗\n');
fprintf('║ Scenario ║ Algorithm    ║ Succ%% ║  Time (mean±std)  ║  Len  (mean±std)  ║\n');
fprintf('╠══════════╬══════════════╬════════╬══════════════════╬═══════════════════╣\n');

prev_sc = -1;
for s = 1:n_sc
    if stats(s,1).scenario ~= prev_sc
        if s > 1
            fprintf('╠══════════╬══════════════╬════════╬══════════════════╬═══════════════════╣\n');
        end
        prev_sc = stats(s,1).scenario;
    end
    for a = 1:n_alg
        st = stats(s, a);
        sc_label = '';
        if a == 1
            idx = find([1 2 3 4] == st.scenario, 1);
            if isempty(idx), sc_label = sprintf('Sc%d', st.scenario);
            else, sc_label = sc_names{idx}; end
        end
        len_str = sprintf('%.3f±%.3f', nandefault(st.length_mean, NaN), nandefault(st.length_std, NaN));
        fprintf('║ %-8s ║ %-12s ║ %5.1f%% ║ %6.3f ± %6.3f s ║ %-17s ║\n', sc_label, st.algorithm, st.success_rate, st.time_mean, st.time_std, len_str);
    end
end

fprintf('╚══════════╩══════════════╩════════╩══════════════════╩═══════════════════╝\n');
fprintf('\n');

end

function v = nandefault(x, d)
    if isnan(x), v = d; else, v = x; end
end
