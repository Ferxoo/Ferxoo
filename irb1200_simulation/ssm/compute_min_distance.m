function [min_dist, closest_body] = compute_min_distance(robot, q, op_pos)

narginchk(3, 3);

assert(isa(robot, 'rigidBodyTree'), 'compute_min_distance: robot must be a rigidBodyTree.');
assert(isnumeric(q) && numel(q) == 6, 'compute_min_distance: q must be a 6-element vector [rad]. Got %d elements.', numel(q));
assert(isnumeric(op_pos) && numel(op_pos) == 3, 'compute_min_distance: op_pos must be a 3-element position vector [m].');

q      = q(:);       
op_pos = op_pos(:);  

R_OP = 0.150; 

BODY_NAMES  = {'link_1','link_2','link_3','link_4','link_5','link_6','tool0'};
LINK_RADII  = [0.055,   0.055,   0.050,   0.040,   0.038,   0.030,   0.020];

min_dist     = Inf;
closest_body = '';

for k = 1:numel(BODY_NAMES)
    body_name = BODY_NAMES{k};

    try
        T_body   = getTransform(robot, q, body_name);
    catch
        continue; 
    end
    body_origin = T_body(1:3, 4); 

    d_centre = norm(body_origin - op_pos);

    d_surface = d_centre - LINK_RADII(k) - R_OP;

    if d_surface < min_dist
        min_dist     = d_surface;
        closest_body = body_name;
    end
end

min_dist = max(0.0, min_dist);

end
