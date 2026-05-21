%% ─────────────────────────────────────────────────────────────
% File        : get_operator_position.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Returns the human operator's position at time t [s]
%               according to the requested motion mode. Used by
%               execute_with_ssm to drive the SSM state machine.
% Inputs      : t    — scalar double ≥ 0, simulation time [s]
%               mode — char: 'static' | 'approach' | 'lateral' |
%                            'random' | 'slow_then_stop'
%                            (default = 'approach')
% Outputs     : pos  — 1×3 double [x, y, z] in world frame [m]
% Dependencies: none (uses persistent variables for 'random' mode)
%% ─────────────────────────────────────────────────────────────

function pos = get_operator_position(t, mode)

narginchk(1, 2);

% Default motion mode
if nargin < 2 || isempty(mode)
    mode = 'approach';
end

assert(isscalar(t) && isnumeric(t) && t >= 0, ...
    'get_operator_position: t must be a non-negative scalar [s]. Got t=%g.', t);
assert(ischar(mode) || isstring(mode), ...
    'get_operator_position: mode must be a char or string.');
mode = char(mode);

switch lower(mode)

    %% ── Static: operator stays safely outside all SSM zones ──────────
    case 'static'
        % Fixed position at 0.70 m from base, offset laterally.
        % Distance to robot base ≈ sqrt(0.7²+0.3²) ≈ 0.76 m — always NORMAL.
        pos = [0.700, 0.300, 0.7];

    %% ── Approach: operator walks along X toward the robot ────────────
    case 'approach'
        % Starts at x=1.5 m, moves at ~0.115 m/s toward the robot base.
        % Enters SLOW zone (d≤1.0 m) at t≈4.4 s.
        % Enters STOP zone (d≤0.5 m) at t≈8.7 s.
        % Clamped to x=0.35 m so the operator cannot reach the robot base.
        x   = max(0.35, 1.5 - 0.115 * t);
        pos = [x, 0.0, 0.7];

    %% ── Lateral: operator moves side-to-side at safe fixed distance ──
    case 'lateral'
        % Maintains x=0.70 m (just outside SLOW zone) while oscillating
        % in Y with a 0.5 m amplitude and 0.3 rad/s angular frequency.
        % Useful for testing hysteresis at the NORMAL/SLOW boundary.
        pos = [0.700, 0.5 * sin(0.3 * t), 0.7];

    %% ── Random: operator takes a small random step every second ──────
    case 'random'
        % Uses persistent variables so successive calls with the SAME t
        % return the same position (deterministic within one simulation).
        % The operator takes one step per second in a bounded region.
        persistent rnd_last_pos rnd_step_t
        if isempty(rnd_last_pos)
            rnd_last_pos = [1.2, 0.4, 0];
            rnd_step_t   = 0.0;
        end

        if (t - rnd_step_t) >= 1.0
            % Small random displacement in XY plane (no Z component)
            delta          = 0.05 * (2 * rand(1, 3) - 1);
            delta(3)       = 0;   % keep operator on the floor
            rnd_last_pos   = rnd_last_pos + delta;
            % Clamp to valid region: x∈[0.30, 1.50], y∈[-0.80, 0.80]
            rnd_last_pos(1) = max(0.30, min(1.50, rnd_last_pos(1)));
            rnd_last_pos(2) = max(-0.80, min(0.80, rnd_last_pos(2)));
            rnd_step_t      = t;
        end
        pos = rnd_last_pos;

    %% ── Slow-then-stop: staged approach to demonstrate all SSM states ───
    case 'slow_then_stop'
        % Phase 1 (t  0– 4 s): approach 1.5 m → 0.75 m (enter SLOW zone)
        % Phase 2 (t  4– 8 s): hold at 0.75 m (robot slows, keeps moving)
        % Phase 3 (t  8–10 s): advance 0.75 m → 0.52 m (near STOP boundary)
        % Phase 4 (t 10–14 s): hold at 0.52 m (trigger STOP_REPLAN + wait)
        % Phase 5 (t >14  s): retreat at 0.15 m/s back to 1.5 m (robot resumes)
        if t <= 4.0
            x = 1.5 - (0.75) * (t / 4.0);              % 1.50 → 0.75 m
        elseif t <= 8.0
            x = 0.75;                                   % hold in SLOW zone
        elseif t <= 10.0
            x = 0.75 - 0.23 * ((t - 8.0) / 2.0);      % 0.75 → 0.52 m
        elseif t <= 14.0
            x = 0.52;                                   % hold → STOP_REPLAN
        else
            x = min(1.5, 0.52 + 0.15 * (t - 14.0));   % retreat at 0.15 m/s
        end
        pos = [x, 0.0, 0.7];

    otherwise
        error(['get_operator_position: unknown mode "%s". ' ...
               'Valid modes: ''static'', ''approach'', ''lateral'', ''random'', ''slow_then_stop''.'], mode);
end

end
