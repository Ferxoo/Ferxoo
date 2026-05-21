%% ─────────────────────────────────────────────────────────────
% File        : draw_ssm_timeline.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Deterministic timeline / chronogram of the slow_then_stop
%               operator mode (Fig 4.8). Shows:
%                 (top)    Operator X position vs time with phase bands
%                 (bottom) SSM state (NORMAL/SLOW/STOP_REPLAN) vs time
%               derived analytically from get_operator_position formulas,
%               so the figure is reproducible without running the simulation.
%               Saved as results/figures/ssm_timeline.png (Fig 4.8).
% Inputs      : none
% Outputs     : none (figure saved as side-effect)
% Dependencies: none (pure MATLAB graphics)
%% ─────────────────────────────────────────────────────────────

function draw_ssm_timeline()

narginchk(0, 0);

% SSM thresholds (must match ssm_state_machine.m)
D_SLOW_ENTER = 1.00;
D_SLOW_EXIT  = 1.05;
D_STOP_ENTER = 0.50;
D_STOP_EXIT  = 0.55;

% Time axis
dt  = 0.02;
t   = 0 : dt : 25.0;

%% Compute operator X position (slow_then_stop formula) ────────
x_op = zeros(size(t));
for k = 1:numel(t)
    tk = t(k);
    if tk <= 4.0
        x_op(k) = 1.5 - 0.75 * (tk / 4.0);
    elseif tk <= 8.0
        x_op(k) = 0.75;
    elseif tk <= 10.0
        x_op(k) = 0.75 - 0.23 * ((tk - 8.0) / 2.0);
    elseif tk <= 14.0
        x_op(k) = 0.52;
    else
        x_op(k) = min(1.5, 0.52 + 0.15 * (tk - 14.0));
    end
end

%% Approximate d_min = x_op (operator on X axis, robot base at origin)
% This is the distance from the base; actual surface distance is slightly
% less due to robot link geometry, but matches the threshold logic shown.
d_min = x_op;

%% Derive SSM state from d_min with hysteresis ─────────────────
ssm_num = ones(size(t));   % 1=NORMAL, 2=SLOW, 3=STOP_REPLAN
prev = 1;
for k = 1:numel(t)
    d = d_min(k);
    if prev == 1       % currently NORMAL
        if d <= D_SLOW_ENTER,  prev = 2; end
    elseif prev == 2   % currently SLOW
        if d <= D_STOP_ENTER,  prev = 3;
        elseif d >= D_SLOW_EXIT, prev = 1; end
    else               % currently STOP_REPLAN
        if d >= D_STOP_EXIT,   prev = 2; end
    end
    ssm_num(k) = prev;
end

%% Phase boundaries for annotation ────────────────────────────
phases = [0, 4, 8, 10, 14, 25];
phase_labels = {'Ph.1', 'Ph.2', 'Ph.3', 'Ph.4', 'Ph.5'};
phase_colors = [0.80,1.00,0.80; 1.00,1.00,0.75; 1.00,0.88,0.70; ...
                1.00,0.75,0.75; 0.80,1.00,0.80];

%% Figure ─────────────────────────────────────────────────────
fig = figure('Name', 'SSM Timeline — slow_then_stop Mode', ...
             'NumberTitle', 'off', 'Position', [100, 100, 1100, 580]);

%% ── Top panel: operator position ─────────────────────────────
ax1 = subplot(2, 1, 1);
hold(ax1, 'on');

% Phase background bands
for p = 1:numel(phase_labels)
    patch(ax1, [phases(p), phases(p+1), phases(p+1), phases(p)], ...
               [0.20, 0.20, 1.75, 1.75], phase_colors(p,:), ...
               'FaceAlpha', 0.35, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    text(ax1, (phases(p)+phases(p+1))/2, 1.65, phase_labels{p}, ...
         'HorizontalAlignment', 'center', 'FontSize', 9, ...
         'Color', [0.3,0.3,0.3], 'FontWeight', 'bold');
end

% SSM threshold horizontal lines
yline(ax1, D_SLOW_ENTER, '--', 'Color', [0.90,0.50,0.00], 'LineWidth', 1.4, ...
      'Label', 'D_{SLOW} = 1.0 m', 'LabelVerticalAlignment', 'bottom', ...
      'DisplayName', 'D_{SLOW\_ENTER}');
yline(ax1, D_STOP_ENTER, '-',  'Color', [0.80,0.10,0.10], 'LineWidth', 1.4, ...
      'Label', 'D_{STOP} = 0.5 m', 'LabelVerticalAlignment', 'top', ...
      'DisplayName', 'D_{STOP\_ENTER}');

% Operator position
plot(ax1, t, x_op, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Operator x(t) [m]');

ylabel(ax1, 'x_{op} [m]', 'FontSize', 11);
xlim(ax1, [0, 25]);  ylim(ax1, [0.20, 1.75]);
legend(ax1, 'Location', 'northeast', 'FontSize', 9);
title(ax1, {'slow\_then\_stop Mode — Operator Position and SSM Safety Thresholds'}, ...
      'FontSize', 12, 'FontWeight', 'bold');
grid(ax1, 'on');  hold(ax1, 'off');

%% ── Bottom panel: SSM state chronogram ───────────────────────
ax2 = subplot(2, 1, 2);
hold(ax2, 'on');

STATE_Y   = [1, 2, 3];   % NORMAL=1, SLOW=2, STOP_REPLAN=3
STATE_CLR = {[0.75,1.0,0.75], [1.0,0.95,0.65], [1.0,0.75,0.75]};
STATE_TXT = {'NORMAL', 'SLOW', 'STOP\_REPLAN'};

% Horizontal band for each state region
dt_half = dt/2;
for k = 1:numel(t)
    s = ssm_num(k);
    patch(ax2, [t(k)-dt_half, t(k)+dt_half, t(k)+dt_half, t(k)-dt_half], ...
               [0.6, 0.6, 3.4, 3.4], STATE_CLR{s}, ...
               'FaceAlpha', 0.85, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

% State label lines
for s = 1:3
    yline(ax2, STATE_Y(s), 'k:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    text(ax2, 0.3, STATE_Y(s), STATE_TXT{s}, 'FontSize', 10, ...
         'FontWeight', 'bold', 'VerticalAlignment', 'middle');
end

% Phase markers
for p = 2:numel(phases)-1
    xline(ax2, phases(p), ':', 'Color', [0.5,0.5,0.5], 'LineWidth', 1.0, ...
          'HandleVisibility', 'off');
end

yticks(ax2, [1, 2, 3]);
yticklabels(ax2, {'NORMAL', 'SLOW', 'STOP'});
xlim(ax2, [0, 25]);  ylim(ax2, [0.5, 3.5]);
xlabel(ax2, 'Time [s]', 'FontSize', 11);
ylabel(ax2, 'SSM State', 'FontSize', 11);
title(ax2, 'SSM State Sequence — NORMAL → SLOW → STOP\_REPLAN → SLOW → NORMAL', ...
      'FontSize', 11);
grid(ax2, 'on');  hold(ax2, 'off');

%% Phase description annotations ──────────────────────────────
phase_desc = { ...
    'Approach', ...
    'Hold SLOW', ...
    'Near STOP', ...
    'STOP + Replan', ...
    'Retreat'};
for p = 1:numel(phase_labels)
    xm = (phases(p) + phases(p+1)) / 2;
    text(ax2, xm, 0.70, phase_desc{p}, ...
         'HorizontalAlignment', 'center', 'FontSize', 8, ...
         'Color', [0.25,0.25,0.25]);
end

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'ssm_timeline');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_ssm_timeline] Figure saved: %s.png\n', out_base);

end
