function obstacles = scenario_central()

obs.name     = 'central_box';
obs.geometry = collisionBox(0.180, 0.350, 0.450);
obs.T_world  = trvec2tform([0.320, 0.000, 0.300]);
obs.color    = [0.80, 0.20, 0.20];

obs.geometry.Pose = obs.T_world;

obstacles = {obs};

end
