function obstacles = build_scenario(scenario_id)

narginchk(1, 1);
assert(isnumeric(scenario_id) && isscalar(scenario_id), 'build_scenario: scenario_id must be a numeric scalar.');

switch scenario_id
    case 1
        obstacles = scenario_free_space();
        label = 'Free space (no obstacles)';
    case 2
        obstacles = scenario_central();
        label = 'Central box obstacle';
    case 3
        obstacles = scenario_corridor();
        label = 'Narrow corridor (two walls)';
    case 4
        obstacles = scenario_dense();
        label = 'Dense (5 mixed obstacles)';
    otherwise
        error('build_scenario: scenario_id must be 1, 2, 3, or 4. Got %d.', scenario_id);
end

fprintf('[build_scenario] Scenario %d loaded: "%s" (%d obstacle(s)).\n', scenario_id, label, numel(obstacles));

end
