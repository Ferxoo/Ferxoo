%% ─────────────────────────────────────────────────────────────
% File        : scenario_central.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Scenario 2 — single box obstacle centred at x=0.320 m,
%               sized to block the arm sweep between the benchmark
%               start/goal configurations (q=[0,-60,60,0,60,0] deg
%               and q=[0,60,-60,0,-60,0] deg).
% Inputs      : none
% Outputs     : obstacles — 1×1 cell array of obstacle structs
% Dependencies: none
%% ─────────────────────────────────────────────────────────────

function obstacles = scenario_central()

% [Change 2.2] Box repositioned to x=0.320 m and enlarged to 180×350×450 mm
% so it intercepts the arm links when sweeping between the new benchmark
% start/goal configs (EE from [0.30,+0.20,0.55] to [0.30,-0.20,0.25] m).
obs.name     = 'central_box';
obs.geometry = collisionBox(0.180, 0.350, 0.450);
obs.T_world  = trvec2tform([0.320, 0.000, 0.300]);
obs.color    = [0.80, 0.20, 0.20];   % red

obs.geometry.Pose = obs.T_world;

obstacles = {obs};

end
