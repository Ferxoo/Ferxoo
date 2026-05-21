%% ─────────────────────────────────────────────────────────────
% File        : visualize_all_scenarios.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Renders all four benchmark scenarios, each in its own
%               figure window. Each figure shows the goal configuration
%               (solid, grey — NORMAL state) and the start configuration
%               overlaid as a ghost (orange, semi-transparent). EE
%               start/goal positions are marked with distinct 3-D markers.
%               Uses the same CONFIGS as run_benchmark.m for consistency.
%               Each figure is saved to results/figures/scenario_N.png
%               using an absolute path derived from mfilename('fullpath').
% Inputs      : robot — rigidBodyTree (DataFormat='column')
% Outputs     : none (figure saved as side-effect)
% Dependencies: build_scenario, draw_scene, getTransform (RST)
%% ─────────────────────────────────────────────────────────────

function visualize_all_scenarios(robot)

narginchk(1, 1);

%% Start/goal configurations — identical to run_benchmark.m ───
Q_START = deg2rad([-90; 70; 0; 0; 0; 0]);
Q_GOAL  = deg2rad([ 90; 70; 0; 0; 0; 0]);

CONFIGS = {
    {Q_START, Q_GOAL};   % Scenario 1 — free space
    {Q_START, Q_GOAL};   % Scenario 2 — central box
    {Q_START, Q_GOAL};   % Scenario 3 — corridor
    {Q_START, Q_GOAL};   % Scenario 4 — dense obstacles
};

SC_NAMES = {'Scenario 1 — Free space', ...
            'Scenario 2 — Central box', ...
            'Scenario 3 — Corridor', ...
            'Scenario 4 — Dense'};

% Operator placed far away so SSM colouring stays at NORMAL (grey)
OP_FAR = [2.0, 0.0, 0.0];

%% Output directory ────────────────────────────────────────────
this_dir = fileparts(mfilename('fullpath'));
fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end

%% One figure per scenario ─────────────────────────────────────
for sc = 1:4
    q_start   = CONFIGS{sc}{1};
    q_goal    = CONFIGS{sc}{2};
    obstacles = build_scenario(sc);

    fig = figure('Name', SC_NAMES{sc}, 'NumberTitle', 'off', ...
                 'Position', [40 + (sc-1)*60, 40 + (sc-1)*40, 900, 650]);

    %% Draw goal configuration (solid grey — NORMAL state) ────
    ax = draw_scene(robot, q_goal, obstacles, OP_FAR, 'NORMAL');
    hold(ax, 'on');

    %% Overlay start configuration as ghost (orange, transparent)
    prev_patches = findobj(ax, 'Type', 'Patch');

    try
        show(robot, q_start, 'Frames', 'off', ...
             'PreservePlot', true, 'FastUpdate', false);
    catch
        try
            show(robot, q_start, 'Frames', 'off', 'PreservePlot', true);
        catch ME_show
            warning('visualize_all_scenarios: show() failed (Sc%d start): %s', ...
                    sc, ME_show.message);
        end
    end

    all_patches   = findobj(ax, 'Type', 'Patch');
    ghost_patches = setdiff(all_patches, prev_patches);
    if ~isempty(ghost_patches)
        set(ghost_patches, 'FaceColor', [1.00, 0.55, 0.00], ...
                           'FaceAlpha', 0.28, 'EdgeColor', 'none');
    end

    %% Mark EE positions with 3-D markers ─────────────────────
    T_s = getTransform(robot, q_start, 'tool0');
    T_g = getTransform(robot, q_goal,  'tool0');
    p_s = T_s(1:3,4)';
    p_g = T_g(1:3,4)';

    plot3(ax, p_s(1), p_s(2), p_s(3), 'o', ...
          'MarkerSize', 9, 'MarkerFaceColor', [1.00, 0.55, 0.00], ...
          'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
          'DisplayName', 'EE start');
    plot3(ax, p_g(1), p_g(2), p_g(3), 'd', ...
          'MarkerSize', 9, 'MarkerFaceColor', [0.10, 0.65, 0.10], ...
          'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
          'DisplayName', 'EE goal');

    title(ax, SC_NAMES{sc}, 'FontSize', 11, 'FontWeight', 'bold');
    hold(ax, 'off');
    T_base = getTransform(robot, q_goal, robot.BaseName);
    cx = T_base(1,4);
    cy = T_base(2,4);
    cz = T_base(3,4);
    
    r = 0.75;
    xlim(ax, [cx - r,  cx + r]);
    ylim(ax, [cy - r,  cy + r]);
    zlim(ax, [cz - 0.1, cz + 1.1]);
    
    campos(ax, [cx + 2.5, cy - 2.5, cz + 1.8]);
    camtarget(ax, [cx, cy, cz + 0.4]);
    camup(ax, [0, 0, 1]);
    camva(ax, 22);

    %% Save each figure individually ──────────────────────────
    out_base = fullfile(fig_dir, sprintf('scenario_%d', sc));
    saveas(fig, [out_base '.png']);
    print(fig, out_base, '-dpng', '-r150');
    fprintf('[visualize_all_scenarios] Sc%d saved: %s.png\n', sc, out_base);
end

end
