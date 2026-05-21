%% ─────────────────────────────────────────────────────────────
% File        : draw_scene.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Renders one simulation frame: robot in current config,
%               all obstacles, the operator sphere, and ISO/TS 15066 SSM
%               safety zone wireframes. Robot patches are coloured by
%               the current SSM state (grey/amber/red).
% Inputs      : robot     — rigidBodyTree (DataFormat='column')
%               q         — 6×1 double, current joint config [rad]
%               obstacles — cell array of obstacle structs
%               op_pos    — 1×3 double, operator world position [m]
%               ssm_state — char: 'NORMAL' | 'SLOW' | 'STOP_REPLAN'
% Outputs     : ax — axes handle
% Dependencies: show (Robotics System Toolbox R2025b)
%% ─────────────────────────────────────────────────────────────

function ax = draw_scene(robot, q, obstacles, op_pos, ssm_state)

narginchk(4, 5);
if nargin < 5 || isempty(ssm_state), ssm_state = 'NORMAL'; end

q      = q(:);
op_pos = op_pos(:)';   % ensure 1×3

%% SSM state colour map ───────────────────────────────────────
switch ssm_state
    case 'NORMAL',      robot_clr = [0.60, 0.60, 0.60];  % grey
    case 'SLOW',        robot_clr = [1.00, 0.70, 0.00];  % amber
    case 'STOP_REPLAN', robot_clr = [0.90, 0.10, 0.10];  % red
    otherwise,          robot_clr = [0.60, 0.60, 0.60];
end

%% Draw robot ─────────────────────────────────────────────────
try
    % FastUpdate=true reuses existing patch graphics objects for animation.
    % PreservePlot=false clears old robot drawing before each frame.
    ax = show(robot, q, 'Frames', 'off', ...
                         'PreservePlot', false, ...
                         'FastUpdate',   true);
catch ME_show
    % Fallback if FastUpdate is not supported on this version
    try
        ax = show(robot, q, 'Frames', 'off', 'PreservePlot', false);
    catch
        warning('draw_scene: show() failed: %s', ME_show.message);
        ax = gca;
        return;
    end
end

% Colour all robot patches according to SSM state
robot_patches = findobj(ax, 'Type', 'Patch');
if ~isempty(robot_patches)
    set(robot_patches, 'FaceColor', robot_clr, 'FaceAlpha', 0.85);
end

hold on;

%% Draw obstacles ─────────────────────────────────────────────
for k = 1:numel(obstacles)
    obs = obstacles{k};
    T   = obs.T_world;
    clr = obs.color;

    if isa(obs.geometry, 'collisionBox')
        dims = [obs.geometry.X, obs.geometry.Y, obs.geometry.Z];
        drawBox(T, dims, clr, 0.45);
    elseif isa(obs.geometry, 'collisionCylinder')
        drawCylinder(T, obs.geometry.Radius, obs.geometry.Length, clr, 0.45);
    elseif isa(obs.geometry, 'collisionSphere')
        drawSphere(T(1:3,4)', obs.geometry.Radius, clr, 0.45);
    end
end

%% Draw operator as blue translucent sphere ───────────────────
R_op = 0.150;   % operator sphere radius [m]
[sx, sy, sz] = sphere(16);
surf(R_op*sx + op_pos(1), R_op*sy + op_pos(2), R_op*sz + op_pos(3), ...
     'FaceColor', [0.20, 0.40, 0.90], 'FaceAlpha', 0.50, ...
     'EdgeColor', 'none', 'DisplayName', 'Operator');

%% Draw SSM safety zone wireframes ────────────────────────────
% Zone radii per ISO/TS 15066 SSM thresholds (measured from robot base)
% Visualisation only: actual SSM uses surface-to-surface distance.
[wx, wy, wz] = sphere(28);
% SLOW zone (1.0 m) — yellow dashed
wf_slow = surf(wx, wy, wz, 'FaceColor', 'none', ...
               'EdgeColor', [1.0, 0.8, 0.0], 'EdgeAlpha', 0.30, ...
               'LineStyle', '--', 'DisplayName', 'SLOW zone (1.0 m)');
% STOP zone (0.5 m) — red solid
surf(0.5*wx, 0.5*wy, 0.5*wz, 'FaceColor', 'none', ...
     'EdgeColor', [0.9, 0.1, 0.1], 'EdgeAlpha', 0.40, ...
     'LineStyle', '-', 'DisplayName', 'STOP zone (0.5 m)');

%% Axes formatting ────────────────────────────────────────────
axis equal;  grid on;
xlabel('X [m]');  ylabel('Y [m]');  zlabel('Z [m]');
view(45, 25);
xlim([-0.30, 1.60]);
ylim([-0.80, 0.80]);
zlim([-0.10, 1.10]);

hold off;

end

%% ── Drawing helpers ──────────────────────────────────────────

function drawBox(T, dims, color, alpha_val)
% Draws a rectangular solid with half-extents dims/2, transformed by T.
    hx = dims(1)/2;  hy = dims(2)/2;  hz = dims(3)/2;
    v = [
        -hx -hy -hz;  hx -hy -hz;  hx  hy -hz; -hx  hy -hz;
        -hx -hy  hz;  hx -hy  hz;  hx  hy  hz; -hx  hy  hz;
    ];
    % Transform vertices to world frame
    vw = (T(1:3,1:3) * v' + repmat(T(1:3,4), 1, 8))';
    f  = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 2 3 7 6; 1 4 8 5];
    patch('Vertices', vw, 'Faces', f, ...
          'FaceColor', color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
end

function drawCylinder(T, radius, height, color, alpha_val)
% Draws a cylinder of given radius and height centred at T, axis along Z.
    [cx, cy, cz] = cylinder(radius, 24);
    cz = cz * height - height/2;   % centre at z=0
    sz = size(cx);
    pts    = [cx(:)'; cy(:)'; cz(:)'; ones(1, numel(cx))];
    pts_w  = T * pts;
    cx_w   = reshape(pts_w(1,:), sz);
    cy_w   = reshape(pts_w(2,:), sz);
    cz_w   = reshape(pts_w(3,:), sz);
    surf(cx_w, cy_w, cz_w, 'FaceColor', color, 'FaceAlpha', alpha_val, ...
         'EdgeColor', 'none');
    % Top and bottom caps
    fill3(cx_w(1,:), cy_w(1,:), cz_w(1,:), color, ...
          'FaceAlpha', alpha_val, 'EdgeColor', 'none');
    fill3(cx_w(2,:), cy_w(2,:), cz_w(2,:), color, ...
          'FaceAlpha', alpha_val, 'EdgeColor', 'none');
end

function drawSphere(centre, radius, color, alpha_val)
% Draws a sphere at centre [1×3] with given radius.
    [sx, sy, sz] = sphere(16);
    surf(radius*sx + centre(1), radius*sy + centre(2), radius*sz + centre(3), ...
         'FaceColor', color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
end
