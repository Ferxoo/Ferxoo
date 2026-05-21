%% ─────────────────────────────────────────────────────────────
% File        : draw_ssm_log.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Visualises the SSM execution log: distance vs time with
%               colour-coded safety zones and a speed-factor subplot.
% Inputs      : log — struct from execute_with_ssm with fields:
%                  .time, .distance, .speed_factor,
%                  .ssm_state (cell of chars), .replan_times
% Outputs     : none (figure + PNG saved as side-effect)
% Dependencies: none (pure MATLAB graphics)
%% ─────────────────────────────────────────────────────────────

function draw_ssm_log(log)

narginchk(1, 1);
assert(isstruct(log) && isfield(log,'time') && isfield(log,'distance'), ...
    'draw_ssm_log: log must be the struct returned by execute_with_ssm.');

% SSM threshold values [m] — must match ssm_state_machine.m
D_SLOW = 1.00;
D_STOP = 0.50;

t    = log.time(:);
dist = log.distance(:);
sf   = log.speed_factor(:);

t_end  = max(t);
d_max  = max(max(dist) * 1.05, D_SLOW * 1.2);

fig = figure('Name', 'SSM Monitor', 'NumberTitle', 'off', ...
             'Position', [100, 100, 1100, 650]);

%% ── Subplot 1: Distance vs time ──────────────────────────────
ax1 = subplot(2, 1, 1);
hold(ax1, 'on');

% Coloured background bands for each zone (drawn first, behind data)
% NORMAL zone (green)
patch(ax1, [0, t_end, t_end, 0], [D_SLOW, D_SLOW, d_max, d_max], ...
      [0.80, 1.00, 0.80], 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
      'HandleVisibility', 'off');
% SLOW zone (yellow)
patch(ax1, [0, t_end, t_end, 0], [D_STOP, D_STOP, D_SLOW, D_SLOW], ...
      [1.00, 1.00, 0.70], 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
      'HandleVisibility', 'off');
% STOP zone (red)
patch(ax1, [0, t_end, t_end, 0], [0, 0, D_STOP, D_STOP], ...
      [1.00, 0.80, 0.80], 'FaceAlpha', 0.30, 'EdgeColor', 'none', ...
      'HandleVisibility', 'off');

% Distance trace
plot(ax1, t, dist, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Robot–Operator distance');

% Threshold horizontal lines
yline(ax1, D_SLOW, '--', 'Color', [0.90, 0.50, 0.00], 'LineWidth', 1.4, ...
      'Label', 'SLOW threshold (1.0 m)', 'LabelVerticalAlignment', 'bottom', ...
      'DisplayName', 'SLOW threshold');
yline(ax1, D_STOP, '-',  'Color', [0.80, 0.10, 0.10], 'LineWidth', 1.4, ...
      'Label', 'STOP threshold (0.5 m)', 'LabelVerticalAlignment', 'top', ...
      'DisplayName', 'STOP threshold');

% Mark replan events with vertical dotted lines
if isfield(log, 'replan_times') && ~isempty(log.replan_times)
    for k = 1:numel(log.replan_times)
        xline(ax1, log.replan_times(k), ':', 'Color', [0.8, 0.0, 0.0], ...
              'LineWidth', 1.2, 'Label', 'Replan', ...
              'HandleVisibility', ternary(k==1,'on','off'), ...
              'DisplayName', 'Replan event');
    end
end

ylabel(ax1, 'Distance [m]', 'FontSize', 10);
xlabel(ax1, 'Time [s]', 'FontSize', 10);
xlim(ax1, [0, t_end]);
ylim(ax1, [0, d_max]);
legend(ax1, 'Location', 'northeast', 'FontSize', 9);
title(ax1, 'Robot–Operator Distance (ISO/TS 15066 SSM)', 'FontSize', 11);
grid(ax1, 'on');
hold(ax1, 'off');

%% ── Subplot 2: Speed factor vs time ──────────────────────────
ax2 = subplot(2, 1, 2);
hold(ax2, 'on');

% Shaded area fill
area(ax2, t, sf, 'FaceColor', [0.30, 0.70, 0.30], 'FaceAlpha', 0.50, ...
     'EdgeColor', 'none');

% Overlay line for clarity
plot(ax2, t, sf, 'Color', [0.10, 0.50, 0.10], 'LineWidth', 1.4);

% Annotate SSM state transitions with text labels
prev_state = '';
for k = 1:numel(log.ssm_state)
    curr_state = log.ssm_state{k};
    if ~strcmp(curr_state, prev_state)
        switch curr_state
            case 'NORMAL',      lbl = 'NORMAL';      clr = [0, 0.5, 0];
            case 'SLOW',        lbl = 'SLOW';         clr = [0.8, 0.5, 0];
            case 'STOP_REPLAN', lbl = 'STOP/REPLAN';  clr = [0.8, 0, 0];
            otherwise,          lbl = curr_state;     clr = [0, 0, 0];
        end
        text(ax2, t(k), 0.92, lbl, 'Color', clr, 'FontSize', 7, ...
             'FontWeight', 'bold', 'Rotation', 90, ...
             'VerticalAlignment', 'top', 'Clipping', 'on');
        prev_state = curr_state;
    end
end

ylim(ax2, [0, 1.10]);
yticks(ax2, [0, 0.25, 0.50, 0.75, 1.00]);
xlim(ax2, [0, t_end]);
ylabel(ax2, 'Speed factor', 'FontSize', 10);
xlabel(ax2, 'Time [s]', 'FontSize', 10);
title(ax2, 'SSM Speed Scaling Factor  (0 = stopped, 1 = full speed)', ...
      'FontSize', 11);
grid(ax2, 'on');
hold(ax2, 'off');

sgtitle('SSM Monitor — Speed and Separation Monitoring (ISO/TS 15066)', ...
        'FontSize', 12, 'FontWeight', 'bold');

%% Save combined figure ────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'ssm_distance_log');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_ssm_log] Figure saved: %s.png\n', out_base);

%% Fig 5.5 — Distance vs time only ────────────────────────────
fig55 = figure('Name', 'SSM: Distance vs Time', 'NumberTitle', 'off', ...
               'Position', [100, 100, 1000, 440]);
ax_d = axes(fig55);
hold(ax_d, 'on');
patch(ax_d, [0, t_end, t_end, 0], [D_SLOW, D_SLOW, d_max, d_max], ...
      [0.80, 1.00, 0.80], 'FaceAlpha', 0.25, 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch(ax_d, [0, t_end, t_end, 0], [D_STOP, D_STOP, D_SLOW, D_SLOW], ...
      [1.00, 1.00, 0.70], 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch(ax_d, [0, t_end, t_end, 0], [0, 0, D_STOP, D_STOP], ...
      [1.00, 0.80, 0.80], 'FaceAlpha', 0.30, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax_d, t, dist, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Robot–Operator distance');
yline(ax_d, D_SLOW, '--', 'Color', [0.90,0.50,0.00], 'LineWidth', 1.4, ...
      'Label', 'D_{SLOW} = 1.0 m', 'LabelVerticalAlignment', 'bottom', ...
      'DisplayName', 'SLOW threshold');
yline(ax_d, D_STOP, '-',  'Color', [0.80,0.10,0.10], 'LineWidth', 1.4, ...
      'Label', 'D_{STOP} = 0.5 m', 'LabelVerticalAlignment', 'top', ...
      'DisplayName', 'STOP threshold');
if isfield(log, 'replan_times') && ~isempty(log.replan_times)
    for k = 1:numel(log.replan_times)
        xline(ax_d, log.replan_times(k), ':', 'Color', [0.8,0,0], 'LineWidth', 1.2, ...
              'Label', 'Replan', 'HandleVisibility', ternary(k==1,'on','off'), ...
              'DisplayName', 'Replan event');
    end
end
ylabel(ax_d, 'd_{min} [m]', 'FontSize', 11);
xlabel(ax_d, 'Time [s]', 'FontSize', 11);
xlim(ax_d, [0, t_end]);  ylim(ax_d, [0, d_max]);
legend(ax_d, 'Location', 'northeast', 'FontSize', 10);
title(ax_d, 'd_{min} vs Time with ISO/TS 15066 SSM Safety Zones', 'FontSize', 12);
grid(ax_d, 'on');  hold(ax_d, 'off');
print(fig55, fullfile(fig_dir, 'ssm_distance_only'), '-dpng', '-r300');
fprintf('[draw_ssm_log] Fig 5.5 saved: ssm_distance_only.png\n');

%% Fig 5.6 — Speed factor + SSM state vs time ─────────────────
fig56 = figure('Name', 'SSM: State & Speed Factor', 'NumberTitle', 'off', ...
               'Position', [100, 100, 1000, 440]);
ax_s = axes(fig56);
hold(ax_s, 'on');
area(ax_s, t, sf, 'FaceColor', [0.30,0.70,0.30], 'FaceAlpha', 0.50, 'EdgeColor', 'none');
plot(ax_s, t, sf, 'Color', [0.10,0.50,0.10], 'LineWidth', 1.4);
prev_state = '';
for k = 1:numel(log.ssm_state)
    curr_state = log.ssm_state{k};
    if ~strcmp(curr_state, prev_state)
        switch curr_state
            case 'NORMAL',      lbl = 'NORMAL';       clr = [0,0.5,0];
            case 'SLOW',        lbl = 'SLOW';          clr = [0.8,0.5,0];
            case 'STOP_REPLAN', lbl = 'STOP/REPLAN';  clr = [0.8,0,0];
            otherwise,          lbl = curr_state;      clr = [0,0,0];
        end
        text(ax_s, t(k), 0.92, lbl, 'Color', clr, 'FontSize', 8, ...
             'FontWeight', 'bold', 'Rotation', 90, ...
             'VerticalAlignment', 'top', 'Clipping', 'on');
        prev_state = curr_state;
    end
end
ylim(ax_s, [0, 1.10]);  yticks(ax_s, [0, 0.25, 0.50, 0.75, 1.00]);
xlim(ax_s, [0, t_end]);
ylabel(ax_s, 'Speed factor', 'FontSize', 11);
xlabel(ax_s, 'Time [s]', 'FontSize', 11);
title(ax_s, 'SSM Speed Scaling Factor and State (0 = stopped, 1 = full speed)', 'FontSize', 12);
grid(ax_s, 'on');  hold(ax_s, 'off');
print(fig56, fullfile(fig_dir, 'ssm_speed_factor'), '-dpng', '-r300');
fprintf('[draw_ssm_log] Fig 5.6 saved: ssm_speed_factor.png\n');

end

%% ── Local helper ─────────────────────────────────────────────
function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
