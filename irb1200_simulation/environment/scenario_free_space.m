%% ─────────────────────────────────────────────────────────────
% File        : scenario_free_space.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Scenario 1 — completely unobstructed workspace.
%               Used as baseline to measure planner overhead and
%               verify that all planners find trivially short paths.
% Inputs      : none
% Outputs     : obstacles — empty 1×0 cell array
% Dependencies: none
%% ─────────────────────────────────────────────────────────────

function obstacles = scenario_free_space()

% No obstacles — free space scenario.
% An empty cell array is returned; all planners should find direct paths.
obstacles = {};

end
