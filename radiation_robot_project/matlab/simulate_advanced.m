function out = simulate_advanced(W, adaptive, use_global, drive)
% ADVANCED mission: removes the planar-model limitations.
%   * 3D terrain: height map -> slope along heading adds gravity term  -m g sin(phi)  (pitch feed-forward in the servo)
%   * motor torque limits: wheel torques saturate, then F and M are recomputed from the limited torques
%   * lidar + occupancy grid (static world, binary) instead of a known obstacle list
%   * two-layer planner: A* on the inflated grid (global) + reactive potential field (local)
%       use_global = false  -> reactive-only (old behaviour; can run into a dead end)
%   * drive = 'wheeled' or 'tracked' (different slip, climbable slope, torque limit)
% Same PID / fuzzy-PID heading controller as simulate_mission.m.
p = robot_params(); a = adv_params();
dr = a.(drive);
noise = @(t) 0.015*(sin(37*t) + sin(91*t + 1))/2;
wrap  = @(q) mod(q + pi, 2*pi) - pi;

obs = W.obstacles; wps = W.waypoints; nW = size(wps,1);
slope = slope_grid(W, a); untrav = slope > dr.slope_max;
occ = false(a.NY, a.NX); terr = false(a.NY, a.NX); seen = false(a.NY, a.NX);
occ_xy = zeros(0,2); path = zeros(0,2); replan_now = true;

x = 0; y = 0; th = 0; v = 0; w = 0; t = 0; step = 0; wp_i = 1;
integ = 0; e_prev = 0; d_f = 0;
hot = false; peak = [0 0 0]; leaks_found = zeros(0,2);
min_clear = 99; path_len = 0; max_tilt = 0; n_sat = 0; collided = false;
L = zeros(15200, 21); k = 0;

while wp_i <= nW && t < 150 && ~collided
    % --- lidar -> occupancy grid (10 Hz) ---
    if mod(step, a.LIDAR_EVERY) == 0
        [occ, terr, seen] = lidar_scan(a, obs, untrav, occ, terr, seen, x, y, th);
        [iy, ix] = find(occ | terr);
        occ_xy = [a.CX(ix(:))', a.CY(iy(:))'];
    end
    goal = wps(wp_i,:);
    % --- global planner (A*) ---
    if use_global && (mod(step, a.REPLAN_EVERY) == 0 || replan_now)
        replan_now = false;
        blocked = inflate_map(a, occ | terr, x, y);
        extra = 8.0*slope.*seen;
        path = astar(a, blocked, extra, cell_of(a, x, y), cell_of(a, goal(1), goal(2)), p.WP_TOL);
    end
    if use_global, target = lookahead_point(a, path, x, y, goal); else, target = goal; end
    % --- local reactive layer ---
    krep = p.K_REP; if use_global, krep = 0.5*p.K_REP; end
    psi_d = reactive_heading(x, y, target, occ_xy, krep, p);
    e = wrap(psi_d - (th + noise(t)));
    % --- (adaptive fuzzy-)PID, identical to simulate_mission.m ---
    d_f = d_f + 0.2*((e - e_prev)/p.DT - d_f); e_prev = e;
    if adaptive, [skp, ski, skd] = fuzzy_scales(e, d_f); else, skp = 1; ski = 1; skd = 1; end
    integ = max(-0.5, min(0.5, integ + e*p.DT));
    u = max(-p.W_MAX, min(p.W_MAX, p.KP0*skp*e + p.KI0*ski*integ + p.KD0*skd*d_f));
    % --- terrain under the robot ---
    [gx, gy] = terrain_grad(W, x, y);
    smag = hypot(gx, gy);
    dzds = gx*cos(th) + gy*sin(th);
    sinphi = dzds/sqrt(1 + dzds^2);
    tilt = atan(smag); max_tilt = max(max_tilt, tilt);
    kappa = 1;
    for i = 1:size(W.slip_zones,1)
        if x >= W.slip_zones(i,1) && x <= W.slip_zones(i,2), kappa = dr.kappa_rubble; end
    end
    kappa = kappa*(1 - a.SLOPE_LOSS*min(smag, 1));
    % --- inner servos -> wheel torques -> saturation -> F, M ---
    F  = p.b*v + p.m*a.G*sinphi + p.Kv*(kappa*p.V_CMD - v);   % friction + gravity (pitch) feed-forward
    Mz = p.c*w + p.Kw*(kappa*u - w);
    tR = p.r*F/2 + p.r*Mz/p.L;  tL = p.r*F/2 - p.r*Mz/p.L;
    tm = dr.tau_max;
    sat = abs(tR) > tm || abs(tL) > tm; n_sat = n_sat + sat;
    tR = max(-tm, min(tm, tR)); tL = max(-tm, min(tm, tL));
    F  = (tR + tL)/p.r;  Mz = p.L/(2*p.r)*(tR - tL);
    vdot = (F - p.b*v - p.m*a.G*sinphi)/p.m;                  % gravity term; the torque limit can break the cancellation
    wdot = (Mz - p.c*w)/p.I;
    % --- debris strike, integrate, move ---
    d_w = 0;
    for i = 1:size(p.STRIKES,1)
        if t >= p.STRIKES(i,1) && t < p.STRIKES(i,1) + p.STRIKE_LEN, d_w = p.STRIKES(i,2); end
    end
    px = x; py = y;
    v = v + p.DT*vdot; w = w + p.DT*wdot;
    th = th + p.DT*(w + d_w);
    x = x + p.DT*v*cos(th); y = y + p.DT*v*sin(th);
    path_len = path_len + hypot(x - px, y - py);
    % --- radiation + leak detection ---
    D = p.DOSE_BG + sum(p.LEAK_S ./ (1 + (hypot(x - W.leaks(:,1), y - W.leaks(:,2))/p.LEAK_W).^2));
    if D > p.THRESH
        if ~hot || D > peak(1), peak = [D x y]; end
        hot = true;
    elseif hot
        leaks_found(end+1,:) = peak(2:3); hot = false; %#ok<AGROW>
    end
    min_clear = min(min_clear, min(hypot(x - obs(:,1), y - obs(:,2)) - obs(:,3) - p.ROBOT_R));
    if min_clear < 0, collided = true; end
    if hypot(goal(1) - x, goal(2) - y) < p.WP_TOL, wp_i = wp_i + 1; path = zeros(0,2); replan_now = true; end
    k = k + 1;
    L(k,:) = [t x y th psi_d e u skp ski skd D wp_i F Mz tR tL v w height_at(W, x, y) rad2deg(tilt) double(sat)];
    t = t + p.DT; step = step + 1;
end
if hot, leaks_found(end+1,:) = peak(2:3); end
L = L(1:k,:);
mapped = 0;
for i = 1:size(obs,1)
    if ~isempty(occ_xy) && min(hypot(occ_xy(:,1) - obs(i,1), occ_xy(:,2) - obs(i,2))) < obs(i,3) + 0.25, mapped = mapped + 1; end
end
out.log = L; out.time = t; out.reached = wp_i - 1; out.leaks = leaks_found;
out.mapped = mapped; out.n_obs = size(obs,1); out.min_clear = min_clear; out.path = path_len;
out.max_tilt = rad2deg(max_tilt); out.sat_pct = 100*n_sat/max(k,1); out.peak_torque = max(max(abs(L(:,15:16))));
out.mean_abs_e = mean(abs(L(:,6))); out.max_abs_e = max(abs(L(:,6)));
out.collided = collided; out.success = (wp_i > nW) && ~collided;
out.slope = slope; out.a = a;
end

% ===================== terrain =====================
function z = height_at(W, x, y)
z = 0;
for i = 1:size(W.mounds,1)
    m = W.mounds(i,:);
    z = z + m(3)*exp(-((x - m(1))^2 + (y - m(2))^2)/(2*m(4)^2));
end
end

function [gx, gy] = terrain_grad(W, x, y)
gx = 0; gy = 0;
for i = 1:size(W.mounds,1)
    m = W.mounds(i,:);
    z  = m(3)*exp(-((x - m(1))^2 + (y - m(2))^2)/(2*m(4)^2));
    gx = gx - z*(x - m(1))/m(4)^2;
    gy = gy - z*(y - m(2))/m(4)^2;
end
end

function s = slope_grid(W, a)
gx = zeros(a.NY, a.NX); gy = gx;
for i = 1:size(W.mounds,1)
    m = W.mounds(i,:);
    z  = m(3)*exp(-((a.CXX - m(1)).^2 + (a.CYY - m(2)).^2)/(2*m(4)^2));
    gx = gx - z.*(a.CXX - m(1))/m(4)^2;
    gy = gy - z.*(a.CYY - m(2))/m(4)^2;
end
s = hypot(gx, gy);
end

% ===================== sensing: lidar -> occupancy grid =====================
function [occ, terr, seen] = lidar_scan(a, obs, untrav, occ, terr, seen, x, y, th)
angs  = th + (0:a.N_RAYS-1)*2*pi/a.N_RAYS;          % 1 x N
steps = 0.2:0.1:3.95;                                % 1 x K  (0.2 ... 3.9 m along each ray)
px = x + cos(angs)'*steps;  py = y + sin(angs)'*steps;   % N x K sample points
inside = false(size(px));
for j = 1:size(obs,1)
    inside = inside | (hypot(px - obs(j,1), py - obs(j,2)) < obs(j,3));
end
hit = any(inside, 2);
[~, first] = max(inside, [], 2);                     % first sample inside an obstacle (the lidar return)
for kk = find(hit)'
    ix = floor((px(kk, first(kk)) - a.X0)/a.RES) + 1;
    iy = floor((py(kk, first(kk)) - a.Y0)/a.RES) + 1;
    if ix >= 1 && ix <= a.NX && iy >= 1 && iy <= a.NY, occ(iy, ix) = true; end
end
near = (a.CXX - x).^2 + (a.CYY - y).^2 < a.LIDAR_R^2;   % slope is sensed inside the same range
terr = terr | (near & untrav);
seen = seen | near;
end

function blocked = inflate_map(a, m, x, y)
[iy, ix] = find(m);
r = ceil(a.INFLATE/a.RES);
blocked = false(a.NY, a.NX);
for dy = -r:r
    for dx = -r:r
        if hypot(dx, dy)*a.RES <= a.INFLATE
            jy = iy + dy; jx = ix + dx;
            ok = jy >= 1 & jy <= a.NY & jx >= 1 & jx <= a.NX;
            blocked(sub2ind([a.NY a.NX], jy(ok), jx(ok))) = true;
        end
    end
end
blocked((a.CXX - x).^2 + (a.CYY - y).^2 < 0.35^2) = false;   % robot's own area is never blocked
end

function c = cell_of(a, x, y)
c = [min(max(floor((x - a.X0)/a.RES) + 1, 1), a.NX), min(max(floor((y - a.Y0)/a.RES) + 1, 1), a.NY)];   % [ix iy]
end

% ===================== global planner: A* =====================
function path = astar(a, blocked, extra, start, goal, wp_tol)
NX = a.NX; NY = a.NY;
sx = start(1); sy = start(2); gx = goal(1); gy = goal(2);
for yy = max(1, gy-3):min(NY, gy+3)                  % a waypoint may sit inside the inflation margin
    for xx = max(1, gx-3):min(NX, gx+3)
        if hypot(xx - gx, yy - gy)*a.RES < wp_tol, blocked(yy, xx) = false; end
    end
end
G = inf(NY, NX); openF = inf(NY, NX); closed = false(NY, NX); parent = zeros(NY, NX);
G(sy, sx) = 0; openF(sy, sx) = hypot(gx - sx, gy - sy);
moves = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
found = false;
while true
    [fm, li] = min(openF(:));
    if isinf(fm), break; end
    [y, x] = ind2sub([NY NX], li);
    openF(y, x) = inf; closed(y, x) = true;
    if x == gx && y == gy, found = true; break; end
    for mm = 1:8
        dx = moves(mm,1); dy = moves(mm,2); nx = x + dx; ny = y + dy;
        if nx < 1 || nx > NX || ny < 1 || ny > NY, continue; end
        if blocked(ny, nx) || closed(ny, nx), continue; end
        if dx ~= 0 && dy ~= 0 && (blocked(y, nx) || blocked(ny, x)), continue; end   % no corner cutting
        ng = G(y, x) + hypot(dx, dy)*(1 + extra(ny, nx));
        if ng < G(ny, nx)
            G(ny, nx) = ng; parent(ny, nx) = li; openF(ny, nx) = ng + hypot(gx - nx, gy - ny);
        end
    end
end
path = zeros(0,2);
if ~found, return; end
cur = sub2ind([NY NX], gy, gx);
while cur ~= 0
    [cy, cx] = ind2sub([NY NX], cur);
    path(end+1,:) = [cx cy]; %#ok<AGROW>
    cur = parent(cy, cx);
end
path = flipud(path);
end

function pt = lookahead_point(a, path, x, y, goal)
if isempty(path), pt = goal; return; end
px = a.CX(path(:,1)); px = px(:); py = a.CY(path(:,2)); py = py(:);
[~, k] = min((px - x).^2 + (py - y).^2);
acc = 0;
for i = k:numel(px)-1
    acc = acc + hypot(px(i+1) - px(i), py(i+1) - py(i));
    if acc >= a.LOOKAHEAD, pt = [px(i+1) py(i+1)]; return; end
end
if hypot(px(end) - x, py(end) - y) > 0.3, pt = [px(end) py(end)]; else, pt = goal; end
end

% ===================== local reactive layer =====================
function psi = reactive_heading(x, y, target, occ_xy, k_rep, p)
dx = target(1) - x; dy = target(2) - y;
dist = max(hypot(dx, dy), 1e-6); ux = dx/dist; uy = dy/dist;
fx = ux; fy = uy;
if ~isempty(occ_xy)
    [c, j] = min(hypot(occ_xy(:,1) - x, occ_xy(:,2) - y));    % nearest mapped cell
    d = c - p.ROBOT_R;
    if d < p.D0 && c > 1e-6
        nx = (x - occ_xy(j,1))/c; ny = (y - occ_xy(j,2))/c;
        tx = -ny; ty = nx;
        if tx*ux + ty*uy < 0, tx = -tx; ty = -ty; end
        mag = k_rep*(1/max(d, 0.05) - 1/p.D0);
        fx = fx + mag*(0.6*nx + 0.8*tx);
        fy = fy + mag*(0.6*ny + 0.8*ty);
    end
end
psi = atan2(fy, fx);
end
