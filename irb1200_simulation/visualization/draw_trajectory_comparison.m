function fig = draw_trajectory_comparison(robot, old_path, new_path, op_pos, save_path)

narginchk(4, 5);
op_pos = op_pos(:)';

fig = figure('Name', 'Trayectoria antes/después de recalcular', 'NumberTitle', 'off', 'Position', [100, 100, 900, 700]);
hold on;

old_xyz = tcp_positions(robot, old_path);
new_xyz = tcp_positions(robot, new_path);

plot3(old_xyz(:,1), old_xyz(:,2), old_xyz(:,3), 'b--o', 'LineWidth', 1.6, 'MarkerSize', 4, 'DisplayName', 'Trayectoria original (antes)');
plot3(new_xyz(:,1), new_xyz(:,2), new_xyz(:,3), 'r-s', 'LineWidth', 1.8, 'MarkerSize', 4, 'DisplayName', 'Trayectoria recalculada (después)');

plot3(old_xyz(1,1), old_xyz(1,2), old_xyz(1,3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8, 'DisplayName', 'Punto de recálculo');

R_op = 0.150;
[sx, sy, sz] = sphere(16);
surf(R_op*sx + op_pos(1), R_op*sy + op_pos(2), R_op*sz + op_pos(3), 'FaceColor', [0.20, 0.40, 0.90], 'FaceAlpha', 0.50, 'EdgeColor', 'none', 'DisplayName', 'Operario');

axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
view(45, 25);
legend('Location', 'best');
title('Trayectoria del TCP: antes vs. después de la recalculación');

hold off;

if nargin >= 5 && ~isempty(save_path)
    saveas(fig, save_path);
end

end

function xyz = tcp_positions(robot, path)
    xyz = zeros(size(path, 1), 3);
    for k = 1:size(path, 1)
        T        = getTransform(robot, path(k, :)', 'tool0');
        xyz(k,:) = T(1:3, 4)';
    end
end
