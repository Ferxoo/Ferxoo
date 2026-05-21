%% ─────────────────────────────────────────────────────────────
% File        : get_joint_limits.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Returns the 6 joint position limits of the ABB IRB 1200
%               5/0.9 in radians. Values match the URDF <limit> tags
%               from abb-noetic-devel (authoritative source).
% Inputs      : none
% Outputs     : limits — 6×2 double [lower_rad, upper_rad]
% Dependencies: none
%% ─────────────────────────────────────────────────────────────

function limits = get_joint_limits()

% ABB IRB 1200 5/90 joint limits — from URDF (irb1200.urdf) and
% ABB product specification 3HAC044038-001.
% Angles in degrees (commented) → converted to radians at runtime.
%
%   Joint  | Lower [°]  | Upper [°]  | Notes
%   -------|-----------|-----------|----------------------------------
%   1      | −170       | +170       | Base rotation (Z axis)
%   2      | −100       | +130       | Shoulder (Y axis)
%   3      | −200       | +70        | Elbow (Y axis) — large lower range
%   4      | −270       | +270       | Wrist roll (X axis)
%   5      | −130       | +130       | Wrist pitch (Y axis)
%   6      | −360       | +360       | Tool flange (X axis)

limits = deg2rad([
    -170,  170;    % joint_1
    -100,  130;    % joint_2
    -200,   70;    % joint_3
    -270,  270;    % joint_4
    -130,  130;    % joint_5
    -360,  360;    % joint_6
]);

end
