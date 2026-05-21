%% ─────────────────────────────────────────────────────────────
% File        : compute_metrics.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Computes quantitative evaluation metrics from a
%               planned joint-space path: total computation time,
%               path length (joint space L2), mean jerk variation.
% Inputs      : path             — N×6 double, joint waypoints [rad]
%               computation_time — scalar double, planner time [s]
%               success          — logical, true if path is valid
% Outputs     : metrics — struct with fields:
%                  .time    — computation time [s]
%                  .length  — path length in joint space [rad]
%                  .jerk    — mean jerk magnitude [rad/s³]
%                  .success — logical
% Dependencies: none (pure MATLAB)
%% ─────────────────────────────────────────────────────────────

function metrics = compute_metrics(path, computation_time, success)

narginchk(3, 3);

%% Guard: failed or trivially short paths ─────────────────────
if ~success || isempty(path) || size(path, 1) < 2
    metrics = struct(...
        'time',    computation_time, ...
        'length',  NaN, ...
        'jerk',    NaN, ...
        'success', false);
    return;
end

%% Assumed uniform time step ──────────────────────────────────
% Planner output has no explicit time stamps. We assume uniform
% spacing of dt=0.05 s per waypoint, which at typical RRT step
% sizes (0.10-0.20 rad) corresponds to ~1-3 rad/s — within limits.
dt = 0.05;   % [s] assumed inter-waypoint time step

%% Path length (joint-space L2 norm) ─────────────────────────
% Sum of Euclidean distances between consecutive 6-DOF configurations.
path_length = sum(vecnorm(diff(path), 2, 2));

%% Jerk variation ─────────────────────────────────────────────
% Three finite-difference passes: position → velocity → acceleration → jerk.
% A lower mean jerk indicates a smoother trajectory, important for
% actuator wear and comfort in human-robot collaboration (ISO/TS 15066).
vel  = diff(path, 1, 1) / dt;        % (N-1)×6  [rad/s]
acc  = diff(vel,  1, 1) / dt;        % (N-2)×6  [rad/s²]
jrk  = diff(acc,  1, 1) / dt;        % (N-3)×6  [rad/s³]
jerk_variation = mean(vecnorm(jrk, 2, 2));  % scalar [rad/s³]

%% Assemble output ────────────────────────────────────────────
metrics.time    = computation_time;
metrics.length  = path_length;
metrics.jerk    = jerk_variation;
metrics.success = true;

end
