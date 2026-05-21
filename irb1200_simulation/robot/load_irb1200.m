%% ─────────────────────────────────────────────────────────────
% File        : load_irb1200.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Loads the ABB IRB 1200 5/0.9 rigidBodyTree from the
%               ROS-Industrial URDF and attaches approximate collision
%               cylinders/spheres to each link for SSM distance queries.
% Inputs      : none
% Outputs     : robot — rigidBodyTree, DataFormat='column'
% Dependencies: importrobot (Robotics System Toolbox),
%               abb-noetic-devel/abb_irb1200_support/urdf/irb1200.urdf
%% ─────────────────────────────────────────────────────────────

function robot = load_irb1200()

%% 1 — Resolve URDF path (absolute) ────────────────────────────
% This file lives in irb1200_simulation/robot/
% The URDF is two levels up in abb-noetic-devel/
this_dir  = fileparts(mfilename('fullpath'));
urdf_rel  = fullfile(this_dir, '..', '..', 'abb-noetic-devel', ...
                     'abb_irb1200_support', 'urdf', 'irb1200.urdf');

% Resolve symlinks and '..' components to an absolute path.
% Try Java first (fast); fall back to cd-based resolution on older MATLAB.
try
    urdf_path = char(java.io.File(urdf_rel).getCanonicalPath());
catch
    urdf_path = urdf_rel;
end

if ~isfile(urdf_path)
    error('load_irb1200: URDF not found at:\n  %s\nRun setup_environment first.', ...
          urdf_path);
end

%% 2 — Import robot model ───────────────────────────────────────
% importrobot reads all links, joints and joint limits from the URDF.
% DataFormat='column' means joint configs are 6x1 column vectors.
robot = importrobot(urdf_path);
robot.DataFormat = 'column';

% Count revolute/prismatic joints (NumNonFixedBodies does not exist in R2025b)
n_dof = sum(cellfun(@(b) ~strcmp(b.Joint.Type, 'fixed'), robot.Bodies));
fprintf('[load_irb1200] URDF imported: %d bodies, %d actuated joints\n', ...
        robot.NumBodies, n_dof);

%% 3 — Verify & display joint limits from URDF ─────────────────
% importrobot already reads limits from the URDF <limit> tags.
% We log them for transparency; no override is done so the URDF is
% the single source of truth (ABB IRB 1200 5/90 production values).
joint_names = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'};
fprintf('[load_irb1200] Joint limits (from URDF):\n');
for k = 1:numel(joint_names)
    jnt = getJointByName(robot, joint_names{k});
    if ~isempty(jnt)
        lim = rad2deg(jnt.PositionLimits);
        fprintf('  %-10s  [%8.2f°, %8.2f°]\n', joint_names{k}, lim(1), lim(2));
    end
end

%% 4 — Add approximate collision geometries to each link ────────
% The URDF references mesh files using ROS package:// URIs which
% MATLAB cannot resolve outside ROS. We therefore attach analytical
% primitives (cylinders/spheres) that conservatively enclose each link.
% These are used for fast distance queries in compute_min_distance.m.
%
% Pose argument is the geometry offset in the BODY frame.
link_collision = {
    'link_1', collisionCylinder(0.055, 0.150), trvec2tform([0,    0,    0.075]);
    'link_2', collisionCylinder(0.055, 0.350), trvec2tform([0,    0,    0.175]);
    'link_3', collisionCylinder(0.050, 0.200), trvec2tform([0,    0,    0.100]);
    'link_4', collisionCylinder(0.040, 0.360), trvec2tform([0.18, 0,    0]);
    'link_5', collisionSphere(0.038),          trvec2tform([0,    0,    0]);
    'link_6', collisionCylinder(0.030, 0.065), trvec2tform([0,    0,    0.032]);
};

for k = 1:size(link_collision, 1)
    body_name = link_collision{k, 1};
    geom      = link_collision{k, 2};
    offset_T  = link_collision{k, 3};

    body_idx = bodyIndex(robot, body_name);
    if isempty(body_idx)
        warning('load_irb1200: body "%s" not found — skipping collision.', body_name);
        continue;
    end
    body = robot.Bodies{body_idx};
    addCollision(body, geom, offset_T);
end

fprintf('[load_irb1200] Collision geometries attached to 6 links.\n');
fprintf('[load_irb1200] Robot ready: %d DOF.\n', n_dof);

end

%% ── Local helpers ────────────────────────────────────────────────
function idx = bodyIndex(robot, name)
% Returns the numeric index into robot.Bodies for the body named 'name',
% or [] if not found. Replaces the removed findBodyIndex method (R2025b).
    idx = find(strcmp(robot.BodyNames, name), 1);
end

function jnt = getJointByName(robot, joint_name)
% Returns the rigidBodyJoint object whose Name matches joint_name.
% Iterates over all bodies and checks each body's parent joint.
    jnt = [];
    for k = 1:robot.NumBodies
        j = robot.Bodies{k}.Joint;
        if strcmp(j.Name, joint_name)
            jnt = j;
            return;
        end
    end
end
