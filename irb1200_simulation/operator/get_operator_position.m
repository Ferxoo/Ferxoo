function pos = get_operator_position(t, mode)

narginchk(1, 2);

if nargin < 2 || isempty(mode)
    mode = 'approach';
end

assert(isscalar(t) && isnumeric(t) && t >= 0, 'get_operator_position: t must be a non-negative scalar [s]. Got t=%g.', t);
assert(ischar(mode) || isstring(mode), 'get_operator_position: mode must be a char or string.');
mode = char(mode);

switch lower(mode)

    case 'static'
        pos = [0.700, 0.300, 0.7];

    case 'approach'
        x   = max(0.35, 1.5 - 0.115 * t);
        pos = [x, 0.0, 0.7];

    case 'lateral'
        pos = [0.700, 0.5 * sin(0.3 * t), 0.7];

    case 'random'
        persistent rnd_last_pos rnd_step_t
        if isempty(rnd_last_pos)
            rnd_last_pos = [1.2, 0.4, 0.7];
            rnd_step_t   = 0.0;
        end

        if (t - rnd_step_t) >= 1.0
            delta          = 0.05 * (2 * rand(1, 3) - 1);
            delta(3)       = 0; 
            rnd_last_pos   = rnd_last_pos + delta;
            rnd_last_pos(1) = max(0.30, min(1.50, rnd_last_pos(1)));
            rnd_last_pos(2) = max(-0.80, min(0.80, rnd_last_pos(2)));
            rnd_step_t      = t;
        end
        pos = rnd_last_pos;

    case 'slow_then_stop'
        if t <= 4.0
            x = 1.5 - (0.75) * (t / 4.0);            
        elseif t <= 8.0
            x = 0.75;                                
        elseif t <= 10.0
            x = 0.75 - 0.23 * ((t - 8.0) / 2.0);   
        elseif t <= 14.0
            x = 0.52;
        else
            x = min(1.5, 0.52 + 0.15 * (t - 14.0));
        end
        pos = [x, 0.0, 0.7];

    otherwise
        error(['get_operator_position: unknown mode "%s". ''Valid modes: ''static'', ''approach'', ''lateral'', ''random'', ''slow_then_stop''.'], mode);
end

end
