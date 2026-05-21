%% ─────────────────────────────────────────────────────────────
% File        : draw_ssm_states.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Generates the SSM state-machine transition diagram (Fig 4.5).
%               Shows the three states (NORMAL, SLOW, STOP_REPLAN) as
%               coloured nodes with annotated transition arrows and the
%               four distance thresholds (D_SLOW_ENTER, D_SLOW_EXIT,
%               D_STOP_ENTER, D_STOP_EXIT). A secondary axis shows the
%               speed_factor profile vs distance.
%               Saved as results/figures/ssm_state_diagram.png (Fig 4.5).
% Inputs      : none
% Outputs     : none (figure saved as side-effect)
% Dependencies: none (pure MATLAB graphics)
%% ─────────────────────────────────────────────────────────────

function draw_ssm_states()

narginchk(0, 0);

% Threshold values — must match ssm_state_machine.m
D_SLOW_ENTER = 1.00;
D_SLOW_EXIT  = 1.05;
D_STOP_ENTER = 0.50;
D_STOP_EXIT  = 0.55;

fig = figure('Name', 'SSM State Machine Diagram', 'NumberTitle', 'off', ...
             'Position', [100, 100, 1100, 520]);

%% ── Left panel: state transition diagram ─────────────────────
ax1 = subplot(1, 2, 1);
hold(ax1, 'on');
axis(ax1, 'off');
xlim(ax1, [0, 10]);  ylim(ax1, [0, 6]);

% State node positions (x_centre, y_centre, radius)
states = struct( ...
    'name',  {'NORMAL',         'SLOW',             'STOP\_REPLAN'}, ...
    'x',     {2.0,              5.0,                8.0          }, ...
    'y',     {3.0,              3.0,                3.0          }, ...
    'color', {[0.75,1.0,0.75],  [1.0,0.95,0.65],   [1.0,0.75,0.75]}, ...
    'tcolor',{[0,0.35,0],       [0.55,0.35,0],      [0.55,0,0]   });

R_node = 0.72;
theta_c = linspace(0, 2*pi, 80);
for k = 1:numel(states)
    cx = states(k).x;  cy = states(k).y;
    fill(ax1, cx + R_node*cos(theta_c), cy + R_node*sin(theta_c), ...
         states(k).color, 'EdgeColor', states(k).tcolor*0.9, 'LineWidth', 2.0);
    text(ax1, cx, cy, states(k).name, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontSize', 11, 'FontWeight', 'bold', ...
         'Color', states(k).tcolor);
end

% Speed factor labels inside nodes
sf_labels = {'sf = 1.0', 'sf = 0.05–1.0', 'sf = 0'};
for k = 1:numel(states)
    text(ax1, states(k).x, states(k).y - 0.28, sf_labels{k}, ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
         'FontSize', 8, 'Color', states(k).tcolor);
end

% --- Transition arrow helper (draw curved arc between nodes)
function drawArrow(ax, x1, y1, x2, y2, label, clr, above)
    mx = (x1+x2)/2;  my = (y1+y2)/2;
    offset = 0.30 * (2*above - 1);   % +0.30 above, -0.30 below
    xc = [x1, mx, x2];  yc = [y1, my+offset, y2];
    xx = interp1(1:3, xc, linspace(1,3,60), 'pchip');
    yy = interp1(1:3, yc, linspace(1,3,60), 'pchip');
    plot(ax, xx, yy, '-', 'Color', clr, 'LineWidth', 1.8);
    % Arrowhead at end
    idx = round(0.85 * numel(xx));
    dx  = xx(end)-xx(idx);  dy = yy(end)-yy(idx);
    d   = norm([dx,dy]);  if d<1e-6, return; end
    dx = dx/d;  dy = dy/d;
    hw = 0.18;  hl = 0.30;
    px = xx(end) - hl*dx;  py = yy(end) - hl*dy;
    patch(ax, [xx(end), px-hw*dy, px+hw*dy], [yy(end), py+hw*dx, py-hw*dx], ...
          clr, 'EdgeColor', clr);
    text(ax, mx, my + offset + 0.22*(2*above-1), label, ...
         'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', clr, ...
         'FontWeight', 'bold');
end

% NORMAL → SLOW  (top arc)
drawArrow(ax1, states(1).x+R_node, states(1).y, ...
               states(2).x-R_node, states(2).y, ...
               sprintf('d \\leq %.2f m', D_SLOW_ENTER), [0.70,0.50,0], true);
% SLOW → NORMAL  (bottom arc, return)
drawArrow(ax1, states(2).x-R_node, states(2).y, ...
               states(1).x+R_node, states(1).y, ...
               sprintf('d > %.2f m', D_SLOW_EXIT), [0,0.45,0], false);
% SLOW → STOP_REPLAN  (top arc)
drawArrow(ax1, states(2).x+R_node, states(2).y, ...
               states(3).x-R_node, states(3).y, ...
               sprintf('d \\leq %.2f m', D_STOP_ENTER), [0.70,0,0], true);
% STOP_REPLAN → SLOW  (bottom arc, return)
drawArrow(ax1, states(3).x-R_node, states(3).y, ...
               states(2).x+R_node, states(2).y, ...
               sprintf('d > %.2f m', D_STOP_EXIT), [0.50,0.30,0], false);

% Replan action label inside STOP_REPLAN
text(ax1, states(3).x, states(3).y+0.30, '(do\_replan)', ...
     'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.55,0,0], ...
     'FontAngle', 'italic');

title(ax1, {'SSM State Machine', 'Transitions and Hysteresis Thresholds'}, ...
      'FontSize', 12, 'FontWeight', 'bold');

%% ── Right panel: speed_factor vs distance ─────────────────────
ax2 = subplot(1, 2, 2);
hold(ax2, 'on');

d_range = linspace(0, 1.30, 500);
sf      = zeros(size(d_range));
for k = 1:numel(d_range)
    d = d_range(k);
    if d > D_SLOW_ENTER
        sf(k) = 1.0;
    elseif d > D_STOP_ENTER
        sf(k) = min(1.0, max(0.05, (d - D_STOP_ENTER)/(D_SLOW_ENTER - D_STOP_ENTER)));
    else
        sf(k) = 0.0;
    end
end

% Background bands
patch(ax2, [D_SLOW_ENTER, 1.30, 1.30, D_SLOW_ENTER], [0,0,1.1,1.1], ...
      [0.80,1.0,0.80], 'FaceAlpha', 0.30, 'EdgeColor', 'none');
patch(ax2, [D_STOP_ENTER, D_SLOW_ENTER, D_SLOW_ENTER, D_STOP_ENTER], [0,0,1.1,1.1], ...
      [1.0,1.0,0.70], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
patch(ax2, [0, D_STOP_ENTER, D_STOP_ENTER, 0], [0,0,1.1,1.1], ...
      [1.0,0.80,0.80], 'FaceAlpha', 0.35, 'EdgeColor', 'none');

plot(ax2, d_range, sf, 'b-', 'LineWidth', 2.2, 'DisplayName', 'speed\_factor');

xline(ax2, D_SLOW_ENTER, '--', 'Color', [0.90,0.50,0.00], 'LineWidth', 1.4, ...
      'Label', sprintf('D_{SLOW\\_ENTER}=%.2f m', D_SLOW_ENTER), ...
      'LabelVerticalAlignment', 'bottom', 'DisplayName', 'D_{SLOW\_ENTER}');
xline(ax2, D_SLOW_EXIT,  ':',  'Color', [0.70,0.40,0.00], 'LineWidth', 1.2, ...
      'Label', sprintf('D_{SLOW\\_EXIT}=%.2f m', D_SLOW_EXIT), ...
      'LabelVerticalAlignment', 'top',    'DisplayName', 'D_{SLOW\_EXIT}');
xline(ax2, D_STOP_ENTER, '-',  'Color', [0.80,0.10,0.10], 'LineWidth', 1.4, ...
      'Label', sprintf('D_{STOP\\_ENTER}=%.2f m', D_STOP_ENTER), ...
      'LabelVerticalAlignment', 'bottom', 'DisplayName', 'D_{STOP\_ENTER}');
xline(ax2, D_STOP_EXIT,  ':',  'Color', [0.60,0.00,0.00], 'LineWidth', 1.2, ...
      'Label', sprintf('D_{STOP\\_EXIT}=%.2f m', D_STOP_EXIT), ...
      'LabelVerticalAlignment', 'top',    'DisplayName', 'D_{STOP\_EXIT}');

% Zone labels
text(ax2, D_SLOW_ENTER+0.02, 0.5, 'NORMAL', 'FontSize', 10, ...
     'Color',[0,0.35,0],'FontWeight','bold','Rotation',0);
text(ax2, (D_STOP_ENTER+D_SLOW_ENTER)/2, 0.5, 'SLOW', 'FontSize', 10, ...
     'Color',[0.55,0.35,0],'FontWeight','bold','HorizontalAlignment','center');
text(ax2, D_STOP_ENTER/2, 0.15, 'STOP', 'FontSize', 10, ...
     'Color',[0.55,0,0],'FontWeight','bold','HorizontalAlignment','center');

xlabel(ax2, 'Distance d_{min} [m]', 'FontSize', 11);
ylabel(ax2, 'speed\_factor', 'FontSize', 11);
title(ax2, {'Speed Scaling vs Distance', ...
            'sf = clamp\left(\frac{d-0.50}{0.50}, 0.05, 1.0\right)'}, ...
      'FontSize', 11);
xlim(ax2, [0, 1.30]);  ylim(ax2, [0, 1.10]);
yticks(ax2, [0, 0.05, 0.25, 0.50, 0.75, 1.00]);
grid(ax2, 'on');  legend(ax2, 'Location', 'southeast', 'FontSize', 9);
hold(ax2, 'off');

sgtitle('SSM State Machine — ISO/TS 15066 Speed and Separation Monitoring', ...
        'FontSize', 13, 'FontWeight', 'bold');

%% Save ─────────────────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end
out_base = fullfile(fig_dir, 'ssm_state_diagram');
print(fig, out_base, '-dpng', '-r300');
fprintf('[draw_ssm_states] Figure saved: %s.png\n', out_base);

end
