function p = robot_params()
% All physical, controller and environment parameters in ONE place.

% ---- rigid body (Newton-Euler) ----
p.m   = 15;      % kg      mass
p.I   = 0.70;    % kg m^2  yaw inertia about the centre (approx. m(a^2+b^2)/12 for 0.62 x 0.42 m chassis)
p.b   = 4.0;     % N s/m   viscous (rolling) friction, translation
p.c   = 0.8;     % N m s/rad viscous friction, rotation
p.r   = 0.10;    % m       wheel radius
p.L   = 0.40;    % m       track width (distance between left and right wheels)

% ---- inner velocity servos (feedback-linearising + friction feed-forward) ----
p.tau_v = 0.20;  % s  closed-loop time constant of the speed servo
p.tau_w = 0.15;  % s  closed-loop time constant of the yaw-rate servo
p.Kv = p.m / p.tau_v;   % N s/m     servo gain  (K_v = m / tau_v)
p.Kw = p.I / p.tau_w;   % N m s/rad servo gain  (K_w = I / tau_w)
p.V_CMD = 0.6;   % m/s  forward speed reference
p.W_MAX = 2.0;   % rad/s yaw-rate command limit

% ---- heading controller base gains + derivative filter ----
p.KP0 = 2.0; p.KI0 = 0.15; p.KD0 = 0.25;
p.Tf  = 0.04;    % s  derivative filter time constant (matches 0.2 smoothing at 10 ms)
p.DT  = 0.01;    % s  control / integration step

% ---- disturbances ----
p.SLIP_ZONES = [6 9; 15 18];            % rubble patches (x-range, m)
p.KAPPA      = 0.6;                     % traction/slip ratio on rubble (1 = clear floor)
p.STRIKES    = [8 1.5; 17 -1.5; 26 1.5; 34 -1.5];   % [time s, yaw-rate kick rad/s]
p.STRIKE_LEN = 0.3;                     % s

% ---- world (unknown to the robot until sensed) ----
p.ROBOT_R = 0.30;                       % m  collision radius
p.OBS = [3 0.6 0.5; 6.5 -1.6 0.6; 7 1.8 0.5; 12 1.0 0.6; 13.5 -2.5 0.5; ...
         14.5 2.0 0.6; 17.5 -0.5 0.7; 19.5 2.2 0.5; 21 -2.0 0.6; 22.5 1.2 0.5];  % x y radius
p.WP    = [5 0; 10 -0.6; 15 0; 20 -0.3; 23.5 -0.5];                              % inspection points
p.LEAKS = [10 -1.2; 15.5 -1.0; 20.5 0.8];                                        % hidden sources
p.LEAK_S = 45; p.LEAK_W = 0.8; p.DOSE_BG = 0.1; p.THRESH = 12;
p.SENSE_R = 3.0; p.WP_TOL = 0.5; p.D0 = 1.5; p.K_REP = 0.45;
end
