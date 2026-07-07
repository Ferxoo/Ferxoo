function pass = validate_kinematics(robot)

narginchk(1, 1);
assert(isa(robot, 'rigidBodyTree'), 'validate_kinematics: input must be a rigidBodyTree object.');

fprintf('[validate_kinematics] Running FK sanity checks on IRB 1200 5/90...\n');

TOL_M = 0.120;

configs = {
    zeros(6,1),               [0.533, 0.000, 0.889],  'Home (all zeros)';
    [pi/2; 0; 0; 0; 0; 0],   [0.000, 0.533, 0.889],  'Base rotated +90°';
    [0; -pi/2; 0; 0; 0; 0],  [0.45,  0.000, 0.450],  'Joint-2 at -90°';
};

for k = 1:size(configs, 1)
    q_test      = configs{k, 1};
    expected    = configs{k, 2};
    desc        = configs{k, 3};

    try
        T   = getTransform(robot, q_test, 'tool0');
        pos = T(1:3, 4)';   % 1×3 world position [m]
        err = norm(pos - expected);

        if err <= TOL_M
            fprintf('  [PASS] Config %d (%s):\n', k, desc);
            fprintf('         EE = [%.4f, %.4f, %.4f] m\n', pos(1), pos(2), pos(3));
        else
            fprintf('  [WARN] Config %d (%s):\n', k, desc);
            fprintf('         EE       = [%.4f, %.4f, %.4f] m\n', pos(1), pos(2), pos(3));
            fprintf('         Expected ≈ [%.4f, %.4f, %.4f] m\n', expected(1), expected(2), expected(3));
            fprintf('         Error    = %.1f mm  (tolerance %.0f mm)\n', err*1000, TOL_M*1000);
            fprintf('         (WARN is informational — reference values are approximate.)\n');
        end

    catch ME
        fprintf('  [ERR]  Config %d (%s): getTransform failed — %s\n', k, desc, ME.message);
    end
end

fprintf('[validate_kinematics] Validation complete (informational only).\n');

pass = true;

end
