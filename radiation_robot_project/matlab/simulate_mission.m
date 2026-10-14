function out = simulate_mission(adaptive)
% Dynamic-route inspection mission.
% Equations of motion (see report):
%   xdot = v cos(th), ydot = v sin(th), thdot = w + d_w(t)
%   m vdot = F - b v,  I wdot = M - c w     (F, M from the inner servos, see robot_accel.m)
% The robot has NO pre-drawn path: it senses obstacles, maps them, and re-plans its desired
% heading psi_d every step; the (adaptive fuzzy-)PID turns the heading error into a yaw-rate command.
p = robot_params();
noise = @(t) 0.015*(sin(37*t) + sin(91*t + 1))/2;
wrap  = @(a) mod(a + pi, 2*pi) - pi;

x = 0; y = 0; th = 0; v = 0; w = 0; t = 0;
integ = 0; e_prev = 0; d_f = 0;
known = false(size(p.OBS,1), 1);
wp_i = 1; leaks_found = zeros(0,2); hot = false; peak = [0 0 0];
min_clear = inf; path_len = 0;
L = zeros(12000, 18); k = 0;

while wp_i <= size(p.WP,1) && t < 120
    % --- sense + map ---
    dist_surf = hypot(x - p.OBS(:,1), y - p.OBS(:,2)) - p.OBS(:,3);
    known = known | (dist_surf < p.SENSE_R);
    % --- dynamic planner -> desired heading ---
    goal  = p.WP(wp_i,:);
    psi_d = plan_heading(x, y, goal, p.OBS, known, p.ROBOT_R, p.D0, p.K_REP);
    e     = wrap(psi_d - (th + noise(t)));
    % --- (adaptive fuzzy-)PID heading controller ---
    d_f   = d_f + 0.2*((e - e_prev)/p.DT - d_f);
    e_prev = e;
    if adaptive
        [skp, ski, skd] = fuzzy_scales(e, d_f);
    else
        skp = 1; ski = 1; skd = 1;
    end
    integ = max(-0.5, min(0.5, integ + e*p.DT));
    u = max(-p.W_MAX, min(p.W_MAX, p.KP0*skp*e + p.KI0*ski*integ + p.KD0*skd*d_f));   % yaw-rate command w_ref
    % --- disturbances ---
    d_w = 0;
    for i = 1:size(p.STRIKES,1)
        if t >= p.STRIKES(i,1) && t < p.STRIKES(i,1) + p.STRIKE_LEN, d_w = p.STRIKES(i,2); end
    end
    kappa = 1;
    for i = 1:size(p.SLIP_ZONES,1)
        if x >= p.SLIP_ZONES(i,1) && x <= p.SLIP_ZONES(i,2), kappa = p.KAPPA; end
    end
    % --- rigid-body dynamics (Newton-Euler + servos), then kinematics ---
    [vdot, wdot, F, M] = robot_accel(v, w, p.V_CMD, u, kappa, p);
    tauR = p.r*F/2 + p.r*M/p.L;          % right wheel torque [N m]
    tauL = p.r*F/2 - p.r*M/p.L;          % left  wheel torque [N m]
    px = x; py = y;
    v  = v  + p.DT*vdot;
    w  = w  + p.DT*wdot;
    th = th + p.DT*(w + d_w);
    x  = x  + p.DT*v*cos(th);
    y  = y  + p.DT*v*sin(th);
    path_len = path_len + hypot(x - px, y - py);
    % --- radiation measurement + leak detection (commit the peak when leaving a hot zone) ---
    D = p.DOSE_BG + sum(p.LEAK_S ./ (1 + (hypot(x - p.LEAKS(:,1), y - p.LEAKS(:,2))/p.LEAK_W).^2));
    if D > p.THRESH
        if ~hot || D > peak(1), peak = [D x y]; end
        hot = true;
    elseif hot
        leaks_found(end+1,:) = peak(2:3); hot = false;
    end
    min_clear = min(min_clear, min(hypot(x - p.OBS(:,1), y - p.OBS(:,2)) - p.OBS(:,3) - p.ROBOT_R));
    if hypot(goal(1) - x, goal(2) - y) < p.WP_TOL, wp_i = wp_i + 1; end
    k = k + 1;
    L(k,:) = [t x y th psi_d e u skp ski skd D min(wp_i, size(p.WP,1)) F M tauR tauL v w];
    t = t + p.DT;
end
if hot, leaks_found(end+1,:) = peak(2:3); end
out.log = L(1:k,:);
out.time = t; out.reached = wp_i - 1; out.leaks = leaks_found; out.known = sum(known);
out.min_clear = min_clear; out.path = path_len;
out.mean_abs_e = mean(abs(out.log(:,6))); out.max_abs_e = max(abs(out.log(:,6)));
out.p = p;
end

function psi = plan_heading(x, y, goal, OBS, known, ROBOT_R, D0, K_REP)
% Goal attraction + repulsion/slide-around from the MAPPED obstacles only.
dx = goal(1) - x; dy = goal(2) - y; dist = hypot(dx, dy);
ux = dx/dist; uy = dy/dist; fx = ux; fy = uy;
for i = find(known)'
    ex = x - OBS(i,1); ey = y - OBS(i,2); c = hypot(ex, ey);
    d = c - OBS(i,3) - ROBOT_R;
    if d < D0
        nx = ex/c; ny = ey/c; tx = -ny; ty = nx;
        if tx*ux + ty*uy < 0, tx = -tx; ty = -ty; end
        mag = K_REP*(1/max(d, 0.05) - 1/D0);
        fx = fx + mag*(0.6*nx + 0.8*tx);
        fy = fy + mag*(0.6*ny + 0.8*ty);
    end
end
psi = atan2(fy, fx);
end
