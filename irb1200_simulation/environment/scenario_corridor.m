function obstacles = scenario_corridor()

obs_left.name     = 'wall_left';
obs_left.geometry = collisionBox(0.060, 0.600, 0.700);
obs_left.T_world  = trvec2tform([0.320, +0.180, 0.350]);
obs_left.color    = [0.80, 0.50, 0.10];

obs_left.geometry.Pose = obs_left.T_world;

obs_right.name     = 'wall_right';
obs_right.geometry = collisionBox(0.060, 0.600, 0.700);
obs_right.T_world  = trvec2tform([0.320, -0.180, 0.350]);
obs_right.color    = [0.80, 0.50, 0.10];

obs_right.geometry.Pose = obs_right.T_world;

obstacles = {obs_left, obs_right};

end
