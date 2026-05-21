%% ─────────────────────────────────────────────────────────────
% File        : draw_convergence.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Plots the convergence of the RRT* path length vs iteration
%               number (Fig 5.7). Requires plan_rrt_star to have been
%               called (at least once) with opts.return_tree=false and
%               info.length_history to be populated. If bench_results is
%               provided (30-run benchmark on Scenario 2) the function
%               averages the convergence curves; otherwise it plots a
%               single run passed via info.
%               Saved as results/figures/rrtstar_convergence.png (Fig 5.7).
% Inputs      : info_or_bench — EITHER: info struct from a single
%                               plan_rrt_star call (must contain
%                               .length_history K×2 [iter, cost])
%                             OR: bench_results struct array from
%                               run_benchmark (contains per-run infos)
%               scenario_id   — (optional) scenario label for title
%                               (default 2)
% Outputs     : none (figure saved as side-effect)
% Dependencies: none (pure MATLAB graphics)
%% ─────────────────────────────────────────────────────────────

function draw_convergence(info_or_bench, scenario_id)

narginchk(1, 2);
if nargin < 2, scenario_id = 2; end

%% Collect convergence histories ──────────────────────────────
histories = {};   % cell array of K×2 matrices

if isstruct(info_or_bench) && isfield(info_or_bench, 'length_history')
    % Single run
    lh = info_or_bench.length_history;
    if ~isempty(lh), histories{end+1} = lh; end

elseif isstruct(info_or_bench) && isfield(info_or_bench, 'infos')
    % bench_results struct: contains per-scenario, per-algorithm cell arrays
    % Attempt to extract RRT* histories for the requested scenario
    sc_idx = scenario_id;
    if sc_idx <= numel(info_or_bench)
        bench_sc = info_or_bench(sc_idx);
        if isfield(bench_sc, 'infos')
            for r = 1:numel(bench_sc.infos)
                inf_r = bench_sc.infos{r};
                if isstruct(inf_r) && isfield(inf_r, 'length_history') && ...
                   ~isempty(inf_r.length_history)
                    histories{end+1} = inf_r.length_history;  %#ok<AGROW>
                end
            end
        end
    end
end

if isempty(histories)
    warning(['draw_convergence: no length_history data found. ' ...
             'Call plan_rrt_star with opts.return_tree=false first.']);
    return;
end

%% Build unified iteration axis (1 : max_iter) ────────────────
max_iter = 0;
for k = 1:numel(histories)
    if ~isempty(histories{k})
        max_iter = max(max_iter, histories{k}(end,1));
    end
end
if max_iter == 0, max_iter = 3000; end

iter_axis = 1:max_iter;

% For each history, create a step-wise cost curve over iter_axis
cost_matrix = NaN(numel(histories), max_iter);
for k = 1:numel(histories)
    lh = histories{k};
    if isempty(lh), continue; end
    cost_cur = NaN;
    hi = 1;
    for it = 1:max_iter
        if hi <= size(lh,1) && lh(hi,1) <= it
            cost_cur = lh(hi,2);
            hi = hi + 1;
        end
        cost_matrix(k, it) = cost_cur;
    end
end

%% Statistics ─────────────────────────────────────────────────
% Only use rows where first improvement was found
valid  = ~all(isnan(cost_matrix), 2);
cm_v   = cost_matrix(valid, :);
if isempty(cm_v)
    warning('draw_convergence: all runs had no improvement recorded.'); return;
end
mean_c = nanmean(cm_v, 1);
std_c  = nanstd(cm_v, 0, 1);
% For std band, only plot from first mean-valid iteration onward
first_valid = find(~isnan(mean_c), 1);

%% Figure ─────────────────────────────────────────────────────
fig = figure('Name', 'RRT* Convergence', 'NumberTitle', 'off', ...
             'Position', [100, 100, 900, 500]);
ax = axes(fig);
hold(ax, 'on');

% Shade ±1σ band
if numel(histories) > 1
    x_band = [iter_axis(first_valid:end), fliplr(iter_axis(first_valid:end))];
    y_up   = mean_c(first_valid:end) + std_c(first_valid:end);
    y_lo   = mean_c(first_valid:end) - std_c(first_valid:end);
    fill(ax, x_band, [y_up, fliplr(y_lo)], [1.0,0.45,0], 'FaceAlpha', 0.20, ...
         'EdgeColor', 'none', 'DisplayName', '\pm 1\sigma band');
end

% Individual run lines (light, thin)
if numel(histories) > 1 && numel(histories) <= 30
    for k = 1:size(cm_v,1)
        idx_start = find(~isnan(cm_v(k,:)), 1);
        if isempty(idx_start), continue; end
        plot(ax, iter_axis(idx_start:end), cm_v(k, idx_start:end), ...
             '-', 'Color', [1.0,0.45,0,0.20], 'LineWidth', 0.8, ...
             'HandleVisibility', 'off');
    end
end

% Mean convergence curve
plot(ax, iter_axis(first_valid:end), mean_c(first_valid:end), '-', ...
     'Color', [1.0,0.40,0.00], 'LineWidth', 2.5, ...
     'DisplayName', sprintf('Mean path length (N=%d runs)', size(cm_v,1)));

% Horizontal dashed line at final mean value
final_cost = mean_c(find(~isnan(mean_c), 1, 'last'));
yline(ax, final_cost, '--', 'Color', [0.4,0.4,0.4], 'LineWidth', 1.2, ...
      'Label', sprintf('Final: %.3f rad', final_cost), ...
      'LabelVerticalAlignment', 'top', 'HandleVisibility', 'off');

xlabel(ax, 'Iteration', 'FontSize', 11);
ylabel(ax, 'Path length L_q [rad]', 'FontSize', 11);
title(ax, {sprintf('RRT* Path Length Convergence — Scenario %d', scenario_id), ...
           'Asymptotic optimisation: path length decreases as tree grows'}, ...
      'FontSize', 12, 'FontWeight', 'bold');
legend(ax, 'Location', 'northeast', 'FontSize', 10);
grid(ax, 'on');
xlim(ax, [1, max_iter]);
hold(ax, 'off');

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'rrtstar_convergence');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_convergence] Figure saved: %s.png\n', out_base);

end
