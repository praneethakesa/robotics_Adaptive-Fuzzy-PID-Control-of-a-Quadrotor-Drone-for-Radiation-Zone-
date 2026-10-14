function a = adv_params()
% Extra parameters for the ADVANCED scenario (3D terrain, lidar map, A* planner, torque limits, drive type).
% Rigid-body, servo, PID and radiation parameters still come from robot_params.m.
a.X0 = -1; a.X1 = 27; a.Y0 = -5; a.Y1 = 5; a.RES = 0.2;          % occupancy-grid extent and cell size [m]
a.NX = round((a.X1 - a.X0)/a.RES); a.NY = round((a.Y1 - a.Y0)/a.RES);
a.CX = a.X0 + ((1:a.NX) - 0.5)*a.RES;                              % cell-centre x (1 x NX)
a.CY = a.Y0 + ((1:a.NY) - 0.5)*a.RES;                              % cell-centre y (1 x NY)
[a.CXX, a.CYY] = meshgrid(a.CX, a.CY);                             % NY x NX
a.LIDAR_R = 4.0; a.N_RAYS = 90; a.LIDAR_EVERY = 10;                % 2D lidar: 90 rays, 4 m, 10 Hz
a.INFLATE = 0.65; a.REPLAN_EVERY = 50; a.LOOKAHEAD = 1.5;          % A*: inflation [m], replan every 0.5 s, look-ahead [m]
a.G = 9.81; a.SLOPE_LOSS = 0.8; a.TILT_LIMIT = 25;                 % gravity, traction lost on slope, tip-over warning [deg]
% drive types: slip ratio on rubble, max climbable slope (rise/run), wheel/track torque limit [N m]
a.wheeled = struct('kappa_rubble', 0.6, 'slope_max', 0.20, 'tau_max', 3.0);
a.tracked = struct('kappa_rubble', 0.8, 'slope_max', 0.45, 'tau_max', 4.0);
end
