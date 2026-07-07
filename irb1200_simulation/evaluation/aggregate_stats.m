function stats = aggregate_stats(results)

narginchk(1, 1);
assert(isstruct(results), 'aggregate_stats: results must be a struct array.');

[n_sc, n_alg] = size(results);

for s_idx = 1:n_sc
    for a_idx = 1:n_alg

        m = results(s_idx, a_idx).metrics;   % 1×n_iter struct array
        n_total = numel(m);

        all_success = [m.success];   % 1×n logical

        all_times   = [m.time];

        succ_lengths = [];
        succ_jerks   = [];
        for i = 1:n_total
            if m(i).success
                succ_lengths(end+1) = m(i).length;  %#ok<AGROW>
                succ_jerks(end+1)   = m(i).jerk;    %#ok<AGROW>
            end
        end

        n_success = sum(all_success);

        stats(s_idx, a_idx).algorithm    = results(s_idx, a_idx).algorithm;
        stats(s_idx, a_idx).scenario     = results(s_idx, a_idx).scenario;
        stats(s_idx, a_idx).n_total      = n_total;
        stats(s_idx, a_idx).n_success    = n_success;
        stats(s_idx, a_idx).success_rate = (n_success / n_total) * 100;  % [%]

        stats(s_idx, a_idx).time_mean   = mean(all_times);
        stats(s_idx, a_idx).time_std    = std(all_times);
        stats(s_idx, a_idx).time_median = median(all_times);

        if ~isempty(succ_lengths)
            stats(s_idx, a_idx).length_mean = mean(succ_lengths);
            stats(s_idx, a_idx).length_std  = std(succ_lengths);
        else
            stats(s_idx, a_idx).length_mean = NaN;
            stats(s_idx, a_idx).length_std  = NaN;
        end

        if ~isempty(succ_jerks)
            stats(s_idx, a_idx).jerk_mean = mean(succ_jerks);
            stats(s_idx, a_idx).jerk_std  = std(succ_jerks);
        else
            stats(s_idx, a_idx).jerk_mean = NaN;
            stats(s_idx, a_idx).jerk_std  = NaN;
        end

    end
end

fprintf('[aggregate_stats] Statistics computed for %d combinations.\n', n_sc * n_alg);

end
