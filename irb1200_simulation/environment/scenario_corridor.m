%% ─────────────────────────────────────────────────────────────
% File        : scenario_corridor.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Scenario 3 — two parallel wall obstacles at x=0.320 m
%               create a 300 mm corridor gap, forcing planners to find
%               the narrow passage between the benchmark start/goal
%               configs (EE from [0.30,+0.20,0.55] to [0.30,-0.20,0.25] m).
% Inputs      : none
% Outputs     : obstacles — 1×2 cell array of obstacle structs
% Dependencies: none
%% ─────────────────────────────────────────────────────────────

function obstacles = scenario_corridor()

% [Change 2.2] Walls moved to x=0.320 m and enlarged to 60×600×700 mm.
% Inner faces at y=±(0.180-0.030)=±0.150 m → corridor gap = 0.300 m.
% The gap is passable but forces planners to route between the walls.

% Left wall: centred at [0.320, +0.180, 0.350]
obs_left.name     = 'wall_left';
obs_left.geometry = collisionBox(0.060, 0.600, 0.700);
obs_left.T_world  = trvec2tform([0.320, +0.180, 0.350]);
obs_left.color    = [0.80, 0.50, 0.10];   % orange

obs_left.geometry.Pose = obs_left.T_world;

% Right wall: centred at [0.320, -0.180, 0.350]
obs_right.name     = 'wall_right';
obs_right.geometry = collisionBox(0.060, 0.600, 0.700);
obs_right.T_world  = trvec2tform([0.320, -0.180, 0.350]);
obs_right.color    = [0.80, 0.50, 0.10];  % orange

obs_right.geometry.Pose = obs_right.T_world;

obstacles = {obs_left, obs_right};

end
