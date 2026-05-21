%% ─────────────────────────────────────────────────────────────
% File        : ssm_state_machine.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : ISO/TS 15066:2016 Speed and Separation Monitoring (SSM)
%               state machine with hysteresis. Determines the safety state
%               and speed scaling factor from the current robot-operator
%               distance and the previous safety state.
% Inputs      : distance   — scalar double ≥ 0, min surface-to-surface
%                            distance between robot and operator [m]
%               prev_state — char: 'NORMAL' | 'SLOW' | 'STOP_REPLAN'
% Outputs     : new_state    — char, updated state
%               speed_factor — scalar double ∈ [0, 1], velocity scaling
%               do_replan    — logical, true only on first entry into
%                              STOP_REPLAN (transition trigger, not tick)
% Dependencies: none
%% ─────────────────────────────────────────────────────────────

function [new_state, speed_factor, do_replan] = ssm_state_machine(distance, prev_state)

narginchk(2, 2);

assert(isnumeric(distance) && isscalar(distance) && distance >= 0, ...
    'ssm_state_machine: distance must be a non-negative scalar [m]. Got %g.', distance);
assert(ischar(prev_state) || isstring(prev_state), ...
    'ssm_state_machine: prev_state must be a char or string.');
prev_state = char(prev_state);

%% SSM zone thresholds (ISO/TS 15066 SSM mode) ─────────────────
D_SLOW_ENTER = 1.00;   % m — NORMAL → SLOW transition distance
D_STOP_ENTER = 0.50;   % m — SLOW  → STOP_REPLAN transition distance
D_SLOW_EXIT  = 1.05;   % m — SLOW  → NORMAL hysteresis offset (+5 cm)
D_STOP_EXIT  = 0.55;   % m — STOP  → SLOW   hysteresis offset (+5 cm)

% Hysteresis prevents state chattering when distance oscillates at a
% threshold boundary. The system requires the operator to move FURTHER
% away than the entry threshold before a less-restrictive state is allowed.

%% State machine ───────────────────────────────────────────────
switch prev_state

    case 'NORMAL'
        if distance <= D_STOP_ENTER
            new_state = 'STOP_REPLAN';   % jumped straight to stop zone
        elseif distance <= D_SLOW_ENTER
            new_state = 'SLOW';
        else
            new_state = 'NORMAL';
        end

    case 'SLOW'
        if distance <= D_STOP_ENTER
            new_state = 'STOP_REPLAN';
        elseif distance >= D_SLOW_EXIT   % hysteresis: must exceed 1.05 m
            new_state = 'NORMAL';
        else
            new_state = 'SLOW';
        end

    case 'STOP_REPLAN'
        if distance >= D_STOP_EXIT       % hysteresis: must exceed 0.55 m
            new_state = 'SLOW';
        else
            new_state = 'STOP_REPLAN';
        end

    otherwise
        % Unknown previous state → reset to NORMAL and re-evaluate
        warning('ssm_state_machine: unknown prev_state "%s". Resetting to NORMAL.', ...
                prev_state);
        [new_state, speed_factor, do_replan] = ssm_state_machine(distance, 'NORMAL');
        return;
end

%% Speed factor computation ────────────────────────────────────
switch new_state
    case 'NORMAL'
        speed_factor = 1.0;

    case 'SLOW'
        % Linear interpolation: v_factor = (d - D_STOP) / (D_SLOW - D_STOP)
        % → 0 at the STOP boundary, 1 at the SLOW boundary.
        % Clamped to [0.05, 1.0] — never allow zero speed in SLOW state
        % (the robot would appear frozen, distinct from STOP_REPLAN).
        speed_factor = (distance - D_STOP_ENTER) / (D_SLOW_ENTER - D_STOP_ENTER);
        speed_factor = min(1.0, max(0.05, speed_factor));

    case 'STOP_REPLAN'
        speed_factor = 0.0;   % full stop, replanning in progress
end

%% Replan trigger ──────────────────────────────────────────────
% do_replan is true ONLY on the transition into STOP_REPLAN, not on
% every tick while in that state. This prevents repeated replan calls.
do_replan = strcmp(new_state, 'STOP_REPLAN') && ~strcmp(prev_state, 'STOP_REPLAN');

end
