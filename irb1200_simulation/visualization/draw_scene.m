function ax = draw_scene(robot, q, obstacles, op_pos, ssm_state, target_ax, view_angle)

narginchk(4, 7);
if nargin < 5 || isempty(ssm_state),  ssm_state  = 'NORMAL';  end
if nargin < 6 || isempty(target_ax),  target_ax  = gca;       end
if nargin < 7 || isempty(view_angle), view_angle = [45, 25];  end

q      = q(:);
op_pos = op_pos(:)'; 

switch ssm_state
    case 'NORMAL',      robot_clr = [0.60, 0.60, 0.60];  
    case 'SLOW',        robot_clr = [1.00, 0.70, 0.00];  
    case 'STOP_REPLAN', robot_clr = [0.90, 0.10, 0.10]; 
    otherwise,          robot_clr = [0.60, 0.60, 0.60];
end

try
    ax = show(robot, q, 'Parent', target_ax, 'Frames', 'off', 'PreservePlot', false, 'FastUpdate',   true);
catch ME_show
    try
        ax = show(robot, q, 'Parent', target_ax, 'Frames', 'off', 'PreservePlot', false);
    catch
        warning('draw_scene: show() failed: %s', ME_show.message);
        ax = target_ax;
        return;
    end
end

robot_patches = findobj(ax, 'Type', 'Patch');
if ~isempty(robot_patches)
    set(robot_patches, 'FaceColor', robot_clr, 'FaceAlpha', 0.85);
end

axes(ax);  
hold on;

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

R_op = 0.150;   
[sx, sy, sz] = sphere(16);
surf(R_op*sx + op_pos(1), R_op*sy + op_pos(2), R_op*sz + op_pos(3), 'FaceColor', [0.20, 0.40, 0.90], 'FaceAlpha', 0.50, 'EdgeColor', 'none', 'DisplayName', 'Operator');

D_SLOW = 1.40;
D_STOP = 0.90;
[wx, wy, wz] = sphere(28);
wf_slow = surf(D_SLOW*wx, D_SLOW*wy, D_SLOW*wz, 'FaceColor', 'none', 'EdgeColor', [1.0, 0.8, 0.0], 'EdgeAlpha', 0.30, 'LineStyle', '--', 'DisplayName', sprintf('SLOW zone (%.1f m)', D_SLOW));
surf(D_STOP*wx, D_STOP*wy, D_STOP*wz, 'FaceColor', 'none', 'EdgeColor', [0.9, 0.1, 0.1], 'EdgeAlpha', 0.40, 'LineStyle', '-', 'DisplayName', sprintf('STOP zone (%.1f m)', D_STOP));

axis equal;  grid on;
xlabel('X [m]');  ylabel('Y [m]');  zlabel('Z [m]');
view(view_angle(1), view_angle(2));
xlim([-1.60, 1.60]);
ylim([-1.60, 1.60]);
zlim([-0.50, 1.60]);

hold off;

end

function drawBox(T, dims, color, alpha_val)
    hx = dims(1)/2;  hy = dims(2)/2;  hz = dims(3)/2;
    v = [
        -hx -hy -hz;  hx -hy -hz;  hx  hy -hz; -hx  hy -hz;
        -hx -hy  hz;  hx -hy  hz;  hx  hy  hz; -hx  hy  hz;
    ];
    vw = (T(1:3,1:3) * v' + repmat(T(1:3,4), 1, 8))';
    f  = [1 2 3 4; 5 6 7 8; 1 2 6 5; 3 4 8 7; 2 3 7 6; 1 4 8 5];
    patch('Vertices', vw, 'Faces', f, 'FaceColor', color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
end

function drawCylinder(T, radius, height, color, alpha_val)
    [cx, cy, cz] = cylinder(radius, 24);
    cz = cz * height - height/2;   % centre at z=0
    sz = size(cx);
    pts    = [cx(:)'; cy(:)'; cz(:)'; ones(1, numel(cx))];
    pts_w  = T * pts;
    cx_w   = reshape(pts_w(1,:), sz);
    cy_w   = reshape(pts_w(2,:), sz);
    cz_w   = reshape(pts_w(3,:), sz);
    surf(cx_w, cy_w, cz_w, 'FaceColor', color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
    fill3(cx_w(1,:), cy_w(1,:), cz_w(1,:), color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
    fill3(cx_w(2,:), cy_w(2,:), cz_w(2,:), color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
end

function drawSphere(centre, radius, color, alpha_val)
    [sx, sy, sz] = sphere(16);
    surf(radius*sx + centre(1), radius*sy + centre(2), radius*sz + centre(3), 'FaceColor', color, 'FaceAlpha', alpha_val, 'EdgeColor', 'none');
end
