function metrics = compute_metrics(path, computation_time, success)

narginchk(3, 3);

if ~success || isempty(path) || size(path, 1) < 2
    metrics = struct(...
        'time',    computation_time, ...
        'length',  NaN, ...
        'jerk',    NaN, ...
        'success', false);
    return;
end

dt = 0.05;  

path_length = sum(vecnorm(diff(path), 2, 2));

vel  = diff(path, 1, 1) / dt;        
acc  = diff(vel,  1, 1) / dt;      
jrk  = diff(acc,  1, 1) / dt;       
jerk_variation = mean(vecnorm(jrk, 2, 2));  

metrics.time    = computation_time;
metrics.length  = path_length;
metrics.jerk    = jerk_variation;
metrics.success = true;

end
