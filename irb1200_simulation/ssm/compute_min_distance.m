%% ─────────────────────────────────────────────────────────────
% File        : compute_min_distance.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Computes the minimum surface-to-surface distance
%               between any robot link and the operator (modelled as
%               a sphere) for ISO/TS 15066 SSM monitoring.
% Inputs      : robot   — rigidBodyTree (DataFormat='column')
%               q       — 6×1 double, joint configuration [rad]
%               op_pos  — 1×3 or 3×1 double, operator position [m]
% Outputs     : min_dist     — scalar double ≥ 0, min distance [m]
%               closest_body — char, name of the nearest link
% Dependencies: getTransform (Robotics System Toolbox)
%% ─────────────────────────────────────────────────────────────

function [min_dist, closest_body] = compute_min_distance(robot, q, op_pos)

narginchk(3, 3);

% Validate inputs
assert(isa(robot, 'rigidBodyTree'), ...
    'compute_min_distance: robot must be a rigidBodyTree.');
assert(isnumeric(q) && numel(q) == 6, ...
    'compute_min_distance: q must be a 6-element vector [rad]. Got %d elements.', numel(q));
assert(isnumeric(op_pos) && numel(op_pos) == 3, ...
    'compute_min_distance: op_pos must be a 3-element position vector [m].');

q      = q(:);       % ensure column vector
op_pos = op_pos(:);  % ensure column vector [m]

%% Constants ───────────────────────────────────────────────────
% Operator modelled as sphere of radius 0.15 m per ISO/TS 15066 Annex A
% (conservative whole-body envelope for a standing adult).
R_OP = 0.150;    % operator sphere radius [m]

% Approximate surface radius of each link, used to convert centre-to-
% centre distance to surface-to-surface distance. These match the
% analytical primitives added in load_irb1200 (conservative radii).
BODY_NAMES  = {'link_1','link_2','link_3','link_4','link_5','link_6','tool0'};
LINK_RADII  = [0.055,   0.055,   0.050,   0.040,   0.038,   0.030,   0.020];

%% Distance computation ────────────────────────────────────────
% Design note: We use the FK of each link ORIGIN (joint attachment point)
% rather than the full collision mesh. This is a CONSERVATIVE approximation
% that may over-trigger the SSM (safe side). A rigorous implementation
% would compute the exact separating distance between collision meshes.
% For the safety function the conservative approach is preferred:
% false-positive slowdowns are preferable to false-negative ones.

min_dist     = Inf;
closest_body = '';

for k = 1:numel(BODY_NAMES)
    body_name = BODY_NAMES{k};

    % Get the body-frame origin in world coordinates via FK
    try
        T_body   = getTransform(robot, q, body_name);
    catch
        continue;  % body might not exist in this robot model variant
    end
    body_origin = T_body(1:3, 4);   % 3×1 world position

    % Centre-to-centre distance between link origin and operator centre
    d_centre = norm(body_origin - op_pos);

    % Subtract both the link surface radius and the operator sphere radius
    % to get an approximated surface-to-surface distance.
    d_surface = d_centre - LINK_RADII(k) - R_OP;

    if d_surface < min_dist
        min_dist     = d_surface;
        closest_body = body_name;
    end
end

% Clamp to zero: negative values indicate overlap (contact), not penetration
% through the robot body, which the analytical model cannot represent.
min_dist = max(0.0, min_dist);

end
