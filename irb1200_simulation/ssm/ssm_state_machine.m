function [new_state, speed_factor, do_replan] = ssm_state_machine(distance, prev_state)

narginchk(2, 2);

assert(isnumeric(distance) && isscalar(distance) && distance >= 0, 'ssm_state_machine: distance must be a non-negative scalar [m]. Got %g.', distance);
assert(ischar(prev_state) || isstring(prev_state), 'ssm_state_machine: prev_state must be a char or string.');
prev_state = char(prev_state);

D_SLOW_ENTER = 1.40;   
D_STOP_ENTER = 0.90;   
D_SLOW_EXIT  = 1.45;   
D_STOP_EXIT  = 0.95;   

switch prev_state

    case 'NORMAL'
        if distance <= D_STOP_ENTER
            new_state = 'STOP_REPLAN';   
        elseif distance <= D_SLOW_ENTER
            new_state = 'SLOW';
        else
            new_state = 'NORMAL';
        end

    case 'SLOW'
        if distance <= D_STOP_ENTER
            new_state = 'STOP_REPLAN';
        elseif distance >= D_SLOW_EXIT   
            new_state = 'NORMAL';
        else
            new_state = 'SLOW';
        end

    case 'STOP_REPLAN'
        if distance >= D_STOP_EXIT     
            new_state = 'SLOW';
        else
            new_state = 'STOP_REPLAN';
        end

    otherwise
        warning('ssm_state_machine: unknown prev_state "%s". Resetting to NORMAL.', prev_state);
        [new_state, speed_factor, do_replan] = ssm_state_machine(distance, 'NORMAL');
        return;
end

switch new_state
    case 'NORMAL'
        speed_factor = 1.0;

    case 'SLOW'
        speed_factor = (distance - D_STOP_ENTER) / (D_SLOW_ENTER - D_STOP_ENTER);
        speed_factor = min(1.0, max(0.05, speed_factor));

    case 'STOP_REPLAN'
        speed_factor = 0.0;  
end

do_replan = strcmp(new_state, 'STOP_REPLAN') && ~strcmp(prev_state, 'STOP_REPLAN');

end
