function draw_benchmark(stats)

narginchk(1, 1);
assert(isstruct(stats), 'draw_benchmark: stats must be a struct array.');

[n_sc, n_alg] = size(stats);

time_mean   = zeros(n_sc, n_alg);
time_std    = zeros(n_sc, n_alg);
len_mean    = zeros(n_sc, n_alg);
len_std     = zeros(n_sc, n_alg);
jerk_mean   = zeros(n_sc, n_alg);
jerk_std    = zeros(n_sc, n_alg);
succ_rate   = zeros(n_sc, n_alg);
alg_names   = cell(1, n_alg);

for a = 1:n_alg
    alg_names{a} = stats(1, a).algorithm;
    for s = 1:n_sc
        st = stats(s, a);
        time_mean(s,a) = nandefault(st.time_mean,   0);
        time_std(s,a)  = nandefault(st.time_std,    0);
        len_mean(s,a)  = nandefault(st.length_mean, 0);
        len_std(s,a)   = nandefault(st.length_std,  0);
        jerk_mean(s,a) = nandefault(st.jerk_mean,   0);
        jerk_std(s,a)  = nandefault(st.jerk_std,    0);
        succ_rate(s,a) = st.success_rate;
    end
end

COLORS = [
    0.20, 0.50, 0.80;   % RRT — blue
    0.20, 0.70, 0.30;   % RRT-Connect — green
    1.00, 0.50, 0.00;   % RRT* — orange
    0.80, 0.20, 0.20;   % APF — red
];
if n_alg > size(COLORS,1)
    COLORS = [COLORS; repmat([0.5,0.5,0.5], n_alg-size(COLORS,1), 1)];
end
COLORS = COLORS(1:n_alg, :);

sc_labels = {'Sc.1 Free', 'Sc.2 Central', 'Sc.3 Corridor', 'Sc.4 Dense'};
sc_labels = sc_labels(1:n_sc);

fig = figure('Name', 'Benchmark Comparison', 'NumberTitle', 'off', 'Position', [50, 50, 1200, 800]);

ax1 = subplot(2, 2, 1);
hb1 = bar(ax1, time_mean, 'grouped');
colorBars(hb1, COLORS);
hold(ax1, 'on');
addErrorBars(ax1, time_mean, time_std, n_alg);
hold(ax1, 'off');
ax1.XTickLabel = sc_labels;
ylabel(ax1, 'Time [s]');
title(ax1, 'Computation Time');
grid(ax1, 'on');

ax2 = subplot(2, 2, 2);
hb2 = bar(ax2, len_mean, 'grouped');
colorBars(hb2, COLORS);
hold(ax2, 'on');
addErrorBars(ax2, len_mean, len_std, n_alg);
hold(ax2, 'off');
ax2.XTickLabel = sc_labels;
ylabel(ax2, 'Path Length [rad]');
title(ax2, 'Path Length (joint space)');
grid(ax2, 'on');

ax3 = subplot(2, 2, 3);
hb3 = bar(ax3, jerk_mean, 'grouped');
colorBars(hb3, COLORS);
hold(ax3, 'on');
addErrorBars(ax3, jerk_mean, jerk_std, n_alg);
hold(ax3, 'off');
ax3.XTickLabel = sc_labels;
ylabel(ax3, 'Mean Jerk [rad/s³]');
title(ax3, 'Jerk Variation (smoothness)');
grid(ax3, 'on');

ax4 = subplot(2, 2, 4);
hb4 = bar(ax4, succ_rate, 'grouped');
colorBars(hb4, COLORS);
ax4.XTickLabel = sc_labels;
ylabel(ax4, 'Success [%]');
ylim(ax4, [0, 110]);
title(ax4, 'Success Rate');
grid(ax4, 'on');

display_names = strrep(alg_names, 'rrt_connect', 'RRT-Connect');
display_names = strrep(display_names, 'rrt_star', 'RRT*');
display_names = strrep(display_names, 'rrt', 'RRT');
display_names = strrep(display_names, 'apf', 'APF');

lgd = legend(ax4, hb4, display_names, 'Location', 'southeast');
lgd.FontSize = 9;

sgtitle('Comparative Benchmark — ABB IRB 1200 | N=30 iterations/combination', 'FontSize', 12, 'FontWeight', 'bold');

this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'benchmark_comparison');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_benchmark] Figure saved: %s.png\n', out_base);

fig52 = figure('Name', 'Benchmark: Computation Time', 'NumberTitle', 'off', 'Position', [100, 100, 800, 480]);
ax = axes(fig52);
hb = bar(ax, time_mean, 'grouped');
colorBars(hb, COLORS);
hold(ax, 'on');
addErrorBars(ax, time_mean, time_std, n_alg);
hold(ax, 'off');
ax.XTickLabel = sc_labels;
ylabel(ax, 't_{comp} [s]', 'FontSize', 11);
xlabel(ax, 'Scenario', 'FontSize', 11);
title(ax, 'Computation Time by Scenario and Algorithm', 'FontSize', 12);
legend(ax, hb, strrep(strrep(strrep(strrep(alg_names, 'rrt_connect','RRT-Connect'),'rrt_star','RRT*'),'rrt','RRT'),'apf','APF'), 'Location', 'northwest', 'FontSize', 10);
grid(ax, 'on');
print(fig52, fullfile(fig_dir, 't_comp_comparison'), '-dpng', '-r300');
fprintf('[draw_benchmark] Fig 5.2 saved: t_comp_comparison.png\n');

fig53 = figure('Name', 'Benchmark: Path Length', 'NumberTitle', 'off', 'Position', [100, 100, 800, 480]);
ax = axes(fig53);
hb = bar(ax, len_mean, 'grouped');
colorBars(hb, COLORS);
hold(ax, 'on');
addErrorBars(ax, len_mean, len_std, n_alg);
hold(ax, 'off');
ax.XTickLabel = sc_labels;
ylabel(ax, 'L_q [rad]', 'FontSize', 11);
xlabel(ax, 'Scenario', 'FontSize', 11);
title(ax, 'Path Length L_q by Scenario and Algorithm', 'FontSize', 12);
legend(ax, hb, strrep(strrep(strrep(strrep(alg_names, 'rrt_connect','RRT-Connect'),'rrt_star','RRT*'),'rrt','RRT'),'apf','APF'), 'Location', 'northwest', 'FontSize', 10);
grid(ax, 'on');
print(fig53, fullfile(fig_dir, 'path_length_comparison'), '-dpng', '-r300');
fprintf('[draw_benchmark] Fig 5.3 saved: path_length_comparison.png\n');

fig54 = figure('Name', 'Benchmark: Success Rate', 'NumberTitle', 'off', 'Position', [100, 100, 800, 480]);
ax = axes(fig54);
hb = bar(ax, succ_rate, 'grouped');
colorBars(hb, COLORS);
ax.XTickLabel = sc_labels;
ylabel(ax, 'P_s [%]', 'FontSize', 11);
xlabel(ax, 'Scenario', 'FontSize', 11);
ylim(ax, [0, 110]);
title(ax, 'Success Rate P_s by Scenario and Algorithm', 'FontSize', 12);
legend(ax, hb, strrep(strrep(strrep(strrep(alg_names, 'rrt_connect','RRT-Connect'),'rrt_star','RRT*'),'rrt','RRT'),'apf','APF'), 'Location', 'southwest', 'FontSize', 10);
grid(ax, 'on');
print(fig54, fullfile(fig_dir, 'success_rate_comparison'), '-dpng', '-r300');
fprintf('[draw_benchmark] Fig 5.4 saved: success_rate_comparison.png\n');

end

function colorBars(hb, COLORS)
    for k = 1:numel(hb)
        hb(k).FaceColor = COLORS(k, :);
        hb(k).EdgeColor = 'none';
    end
end

function addErrorBars(ax, means, stds, n_alg)
    n_sc   = size(means, 1);
    n_bars = n_alg;
    group_w = 0.8;
    bar_w   = group_w / n_bars;

    for a = 1:n_bars
        x_pos = (1:n_sc) - group_w/2 + bar_w*(a-0.5);
        errorbar(ax, x_pos, means(:,a), stds(:,a), 'k.', 'LineWidth', 1.2, 'CapSize', 5, 'HandleVisibility', 'off');
    end
end

function v = nandefault(x, d)
    if isnan(x), v = d; else, v = x; end
end
