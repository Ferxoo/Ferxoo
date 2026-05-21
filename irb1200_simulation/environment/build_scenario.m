%% ─────────────────────────────────────────────────────────────
% File        : build_scenario.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Factory function — returns an obstacle cell array for
%               the requested scenario ID (1–4).
% Inputs      : scenario_id — integer scalar, 1 = free space,
%                             2 = central box, 3 = corridor,
%                             4 = dense (5 obstacles)
% Outputs     : obstacles   — 1×N cell array; each cell is a struct:
%                 .name     — char, human-readable label
%                 .geometry — collision object (collisionBox /
%                             collisionCylinder) with .Pose = T_world
%                 .T_world  — 4×4 double, obstacle pose in world frame
%                 .color    — 1×3 double [R G B], for visualisation
% Dependencies: scenario_free_space, scenario_central,
%               scenario_corridor, scenario_dense
%% ─────────────────────────────────────────────────────────────

function obstacles = build_scenario(scenario_id)

narginchk(1, 1);
assert(isnumeric(scenario_id) && isscalar(scenario_id), ...
    'build_scenario: scenario_id must be a numeric scalar.');

switch scenario_id
    case 1
        obstacles = scenario_free_space();
        label = 'Free space (no obstacles)';
    case 2
        obstacles = scenario_central();
        label = 'Central box obstacle';
    case 3
        obstacles = scenario_corridor();
        label = 'Narrow corridor (two walls)';
    case 4
        obstacles = scenario_dense();
        label = 'Dense (5 mixed obstacles)';
    otherwise
        error('build_scenario: scenario_id must be 1, 2, 3, or 4. Got %d.', ...
              scenario_id);
end

fprintf('[build_scenario] Scenario %d loaded: "%s" (%d obstacle(s)).\n', ...
        scenario_id, label, numel(obstacles));

end
