function obstacles = scenario_dense()

obs1.name     = 'box_rf';
obs1.geometry = collisionBox(0.130, 0.130, 0.450);
obs1.T_world  = trvec2tform([-0.300,  0.150, 0.250]);
obs1.color    = [0.80, 0.20, 0.20];
obs1.geometry.Pose = obs1.T_world;

obs2.name     = 'box_rr';
obs2.geometry = collisionBox(0.120, 0.120, 0.500);
obs2.T_world  = trvec2tform([0.350, -0.120, 0.300]);
obs2.color    = [0.80, 0.20, 0.20];
obs2.geometry.Pose = obs2.T_world;

obs3.name     = 'cyl_centre';
obs3.geometry = collisionCylinder(0.075, 0.500);
obs3.T_world  = trvec2tform([-0.280,  0.000, 0.300]);
obs3.color    = [0.90, 0.50, 0.10];
obs3.geometry.Pose = obs3.T_world;

obs4.name     = 'box_left';
obs4.geometry = collisionBox(0.160, 0.090, 0.400);
obs4.T_world  = trvec2tform([0.250,  0.220, 0.250]);
obs4.color    = [0.90, 0.50, 0.10];
obs4.geometry.Pose = obs4.T_world;

obs5.name     = 'cyl_far';
obs5.geometry = collisionCylinder(0.065, 0.600);
obs5.T_world  = trvec2tform([0.380,  0.080, 0.350]);
obs5.color    = [0.80, 0.20, 0.20];
obs5.geometry.Pose = obs5.T_world;

obstacles = {obs1, obs2, obs3, obs4, obs5};

end
