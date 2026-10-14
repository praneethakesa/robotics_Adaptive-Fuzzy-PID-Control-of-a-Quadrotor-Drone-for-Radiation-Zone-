"""ADVANCED extension of the inspection mission (removes the limitations of the planar model).

Added on top of mission_core.py (the original planar mission is NOT changed):
  1. 3D terrain: height map z(x,y) -> slope along the heading adds a gravity term  -m g sin(phi)
     to the translational equation; tilt angle is logged; slope also reduces traction.
  2. Motor torque limits: wheel torques are saturated, then F and M are recomputed from them.
  3. Lidar + occupancy grid (static world, binary): the map is built from rays, not from a list.
  4. Two-layer planner: A* on the inflated grid (global) + reactive potential field (local).
     'use_global=False' gives the old reactive-only behaviour, which can get trapped in a dead end.
  5. Drive type: 'wheeled' or 'tracked' (different slip, slope capability, torque limit).
Terrain slope is 'sensed' within the lidar range (simplification: a depth sensor reports untraversable cells).
"""
import math, heapq
import numpy as np
from mission_core import PID, wrap, noise, strike_at, STRIKES, DT, V_CMD, W_MAX, ROBOT_R, WP_TOL, \
    LEAK_S, LEAK_W, DOSE_BG, LEAK_THRESH, K_REP, D0

# ---- rigid body + servos (same numbers as MATLAB robot_params.m) ----
MASS, INERTIA, B_FR, C_FR, R_WHEEL, TRACK, GRAV = 15.0, 0.70, 4.0, 0.8, 0.10, 0.40, 9.81
TAU_V, TAU_W = 0.20, 0.15
KV, KW = MASS / TAU_V, INERTIA / TAU_W

DRIVES = {  # kappa on rubble, max climbable slope (rise/run), wheel/track torque limit [N m]
    'wheeled': dict(kappa_rubble=0.6, slope_max=0.20, tau_max=3.0),
    'tracked': dict(kappa_rubble=0.8, slope_max=0.45, tau_max=4.0),
}
SLOPE_TRACTION_LOSS = 0.8      # kappa_eff = kappa * (1 - 0.8 * |slope|)

# ---- map / sensing ----
X0, X1, Y0, Y1, RES = -1.0, 27.0, -5.0, 5.0, 0.2
NX, NY = int(round((X1 - X0) / RES)), int(round((Y1 - Y0) / RES))
LIDAR_R, N_RAYS, LIDAR_EVERY = 4.0, 90, 10        # 10 Hz scan
INFLATE = 0.65                                      # inflation radius for the global planner [m]
REPLAN_EVERY, LOOKAHEAD = 50, 1.5
CX = X0 + (np.arange(NX) + 0.5) * RES
CY = Y0 + (np.arange(NY) + 0.5) * RES
CXX, CYY = np.meshgrid(CX, CY)                      # shape (NY, NX)


# ======================= worlds =======================
def hard_world():
    """Dense field (many obstacles), height mounds, rubble patches, and a U-shaped dead end."""
    obs = [(2.5, 1.2, .5), (3.5, -1.8, .5), (5.5, 3.4, .6), (7.5, -3.5, .6),
           (9.5, 1.6, .5), (10.5, -2.0, .5), (11.5, 0.4, .5), (12.5, 2.6, .6), (12.8, -1.4, .5), (13.5, 1.2, .5)]
    # U-shaped pocket opening toward the robot: back wall at x=20, arms along y=+-2.7; goal is behind it
    for yy in np.arange(-2.7, 2.71, 0.9):
        obs.append((20.0, float(yy), .5))
    for xx in (19.1, 18.2, 17.3, 16.4):
        obs.append((xx, 2.7, .5)); obs.append((xx, -2.7, .5))
    return dict(
        obstacles=obs,
        waypoints=[(4.5, 0.0), (9.0, -0.4), (14.5, -0.5), (24.0, 0.0)],
        leaks=[(9.4, -1.2), (14.6, 0.9), (23.2, 0.6)],
        slip_zones=[(6.0, 9.0), (15.0, 18.0)],
        # height mounds (x, y, height h, width s): gentle ones plus one steep pile that wheels cannot climb
        mounds=[(3.0, -0.3, .20, 1.0), (9.5, -0.3, .20, 1.0), (13.0, -0.4, .20, 1.0), (6.5, 0.2, .55, 0.8)],
    )


def height(world, x, y):
    return sum(h * math.exp(-((x - mx) ** 2 + (y - my) ** 2) / (2 * s * s)) for mx, my, h, s in world['mounds'])


def grad(world, x, y):
    gx = gy = 0.0
    for mx, my, h, s in world['mounds']:
        z = h * math.exp(-((x - mx) ** 2 + (y - my) ** 2) / (2 * s * s))
        gx += -z * (x - mx) / (s * s)
        gy += -z * (y - my) / (s * s)
    return gx, gy


def slope_grid(world):
    gx = np.zeros_like(CXX); gy = np.zeros_like(CXX)
    for mx, my, h, s in world['mounds']:
        z = h * np.exp(-((CXX - mx) ** 2 + (CYY - my) ** 2) / (2 * s * s))
        gx += -z * (CXX - mx) / (s * s); gy += -z * (CYY - my) / (s * s)
    return np.hypot(gx, gy)


def dose_at(world, x, y):
    return DOSE_BG + sum(LEAK_S / (1 + (math.hypot(x - lx, y - ly) / LEAK_W) ** 2) for lx, ly in world['leaks'])


# ======================= sensing: lidar -> occupancy grid =======================
class Map:
    def __init__(self, world, slope_max):
        self.w = world
        self.obs = np.array(world['obstacles'])
        self.occ = np.zeros((NY, NX), bool)            # cells hit by lidar (obstacle surface)
        self.terr = np.zeros((NY, NX), bool)           # sensed untraversable-slope cells
        self.slope = slope_grid(world)
        self.untrav = self.slope > slope_max
        self.seen = np.zeros((NY, NX), bool)           # terrain seen so far
        self.dirty = True

    def scan(self, x, y, th):
        angs = th + np.linspace(0, 2 * np.pi, N_RAYS, endpoint=False)
        steps = np.arange(0.2, LIDAR_R, 0.1)
        px = x + np.cos(angs)[:, None] * steps[None, :]
        py = y + np.sin(angs)[:, None] * steps[None, :]
        d = np.hypot(px[..., None] - self.obs[:, 0], py[..., None] - self.obs[:, 1]) - self.obs[:, 2]
        inside = (d < 0).any(axis=2)
        hit = inside.any(axis=1)
        first = inside.argmax(axis=1)
        for k in np.nonzero(hit)[0]:
            ix = int((px[k, first[k]] - X0) / RES); iy = int((py[k, first[k]] - Y0) / RES)
            if 0 <= ix < NX and 0 <= iy < NY and not self.occ[iy, ix]:
                self.occ[iy, ix] = True; self.dirty = True
        near = (CXX - x) ** 2 + (CYY - y) ** 2 < LIDAR_R ** 2
        newt = near & self.untrav & ~self.terr
        if newt.any():
            self.terr |= newt; self.dirty = True
        self.seen |= near

    def occupied_xy(self):
        m = self.occ | self.terr
        iy, ix = np.nonzero(m)
        return np.column_stack([CX[ix], CY[iy]])

    def blocked(self, x, y):
        """Inflated occupancy for the global planner (robot start area cleared)."""
        m = self.occ | self.terr
        iy, ix = np.nonzero(m)
        r = int(math.ceil(INFLATE / RES))
        out = np.zeros_like(m)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if math.hypot(dx, dy) * RES <= INFLATE:
                    jy, jx = iy + dy, ix + dx
                    ok = (jy >= 0) & (jy < NY) & (jx >= 0) & (jx < NX)
                    out[jy[ok], jx[ok]] = True
        out[(CXX - x) ** 2 + (CYY - y) ** 2 < 0.35 ** 2] = False
        return out


# ======================= global planner: A* =======================
def cell(x, y):
    return min(max(int((x - X0) / RES), 0), NX - 1), min(max(int((y - Y0) / RES), 0), NY - 1)


def astar(blocked, extra, start, goal):
    sx, sy = start; gx, gy = goal
    blocked = blocked.copy()
    for yy in range(max(0, gy - 3), min(NY, gy + 4)):          # a waypoint may sit inside the inflation margin
        for xx in range(max(0, gx - 3), min(NX, gx + 4)):
            if math.hypot(xx - gx, yy - gy) * RES < WP_TOL:
                blocked[yy, xx] = False
    openq = [(0.0, 0.0, sx, sy)]; g = {(sx, sy): 0.0}; par = {}
    moves = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while openq:
        _, gc, x, y = heapq.heappop(openq)
        if (x, y) == (gx, gy):
            path = [(x, y)]
            while (x, y) in par:
                x, y = par[(x, y)]; path.append((x, y))
            return path[::-1]
        if gc > g.get((x, y), 1e9) + 1e-9:
            continue
        for dx, dy in moves:
            nx_, ny_ = x + dx, y + dy
            if not (0 <= nx_ < NX and 0 <= ny_ < NY) or blocked[ny_, nx_]:
                continue
            if dx and dy and (blocked[y, nx_] or blocked[ny_, x]):   # no corner cutting
                continue
            ng = gc + math.hypot(dx, dy) * (1.0 + extra[ny_, nx_])
            if ng < g.get((nx_, ny_), 1e9):
                g[(nx_, ny_)] = ng; par[(nx_, ny_)] = (x, y)
                heapq.heappush(openq, (ng + math.hypot(gx - nx_, gy - ny_), ng, nx_, ny_))
    return []


def lookahead_point(path, x, y, goal):
    if not path:
        return goal
    pts = [(CX[i], CY[j]) for i, j in path]
    k = min(range(len(pts)), key=lambda i: (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2)
    acc = 0.0
    for i in range(k, len(pts) - 1):
        acc += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        if acc >= LOOKAHEAD:
            return pts[i + 1]
    return pts[-1] if math.hypot(pts[-1][0] - x, pts[-1][1] - y) > 0.3 else goal


# ======================= local reactive layer =======================
def reactive_heading(x, y, target, occ_xy, k_rep):
    dx, dy = target[0] - x, target[1] - y
    dist = max(math.hypot(dx, dy), 1e-6)
    ux, uy = dx / dist, dy / dist
    fx, fy = ux, uy
    if len(occ_xy):
        dd = np.hypot(occ_xy[:, 0] - x, occ_xy[:, 1] - y)
        j = int(dd.argmin()); c = dd[j]
        d = c - ROBOT_R
        if d < D0 and c > 1e-6:
            nx_, ny_ = (x - occ_xy[j, 0]) / c, (y - occ_xy[j, 1]) / c
            tx, ty = -ny_, nx_
            if tx * ux + ty * uy < 0:
                tx, ty = -tx, -ty
            mag = k_rep * (1.0 / max(d, 0.05) - 1.0 / D0)
            fx += mag * (0.6 * nx_ + 0.8 * tx); fy += mag * (0.6 * ny_ + 0.8 * ty)
    return math.atan2(fy, fx)


# ======================= mission =======================
def run_adv(world, adaptive=True, use_global=True, drive='wheeled', t_max=150.0, strikes=STRIKES):
    dr = DRIVES[drive]
    mp = Map(world, dr['slope_max'])
    obs = np.array(world['obstacles'])
    wps = world['waypoints']
    pid = PID(adaptive)
    x = y = th = v = w = 0.0
    t, step, wp_i = 0.0, 0, 0
    path, hot, peak, leaks_found = [], False, (0.0, 0.0, 0.0), []
    min_clear, path_len, max_tilt, n_sat = 99.0, 0.0, 0.0, 0
    log = []
    replan_now = True
    occ_xy = np.zeros((0, 2)); blocked = None
    collided = False
    while wp_i < len(wps) and t < t_max and not collided:
        if step % LIDAR_EVERY == 0:
            mp.scan(x, y, th)
            if mp.dirty:
                occ_xy = mp.occupied_xy()
        goal = wps[wp_i]
        if use_global and (step % REPLAN_EVERY == 0 or replan_now):
            replan_now = False
            blocked = mp.blocked(x, y)
            extra = 8.0 * mp.slope * mp.seen
            path = astar(blocked, extra, cell(x, y), cell(*goal))
        target = lookahead_point(path, x, y, goal) if use_global else goal
        psi_d = reactive_heading(x, y, target, occ_xy, K_REP * (0.5 if use_global else 1.0))
        e = wrap(psi_d - (th + noise(t)))
        u = pid.update(e, DT)
        # --- terrain ---
        gx, gy = grad(world, x, y)
        smag = math.hypot(gx, gy)
        dzds = gx * math.cos(th) + gy * math.sin(th)
        sinphi = dzds / math.sqrt(1 + dzds * dzds)
        tilt = math.atan(smag)
        max_tilt = max(max_tilt, tilt)
        kappa = 1.0
        for a, b in world['slip_zones']:
            if a <= x <= b:
                kappa = dr['kappa_rubble']
        kappa *= (1 - SLOPE_TRACTION_LOSS * min(smag, 1.0))
        # --- inner servos -> wheel torques -> saturation -> F, M ---
        F = B_FR * v + MASS * GRAV * sinphi + KV * (kappa * V_CMD - v)   # friction + gravity (IMU pitch) feed-forward
        Mz = C_FR * w + KW * (kappa * u - w)
        tR = R_WHEEL * F / 2 + R_WHEEL * Mz / TRACK
        tL = R_WHEEL * F / 2 - R_WHEEL * Mz / TRACK
        tm = dr['tau_max']
        sat = abs(tR) > tm or abs(tL) > tm
        n_sat += sat
        tR, tL = max(-tm, min(tm, tR)), max(-tm, min(tm, tL))
        F = (tR + tL) / R_WHEEL
        Mz = TRACK / (2 * R_WHEEL) * (tR - tL)
        vdot = (F - B_FR * v - MASS * GRAV * sinphi) / MASS      # gravity term from the height map; torque limit can break the cancellation
        wdot = (Mz - C_FR * w) / INERTIA
        d_w = strike_at(t, strikes)
        px, py = x, y
        v += DT * vdot; w += DT * wdot
        th += DT * (w + d_w)
        x += DT * v * math.cos(th); y += DT * v * math.sin(th)
        path_len += math.hypot(x - px, y - py)
        D = dose_at(world, x, y)
        if D > LEAK_THRESH:
            if not hot or D > peak[0]:
                peak = (D, x, y)
            hot = True
        elif hot:
            leaks_found.append((peak[1], peak[2])); hot = False
        min_clear = min(min_clear, float(np.min(np.hypot(x - obs[:, 0], y - obs[:, 1]) - obs[:, 2] - ROBOT_R)))
        if min_clear < 0:
            collided = True
        if math.hypot(goal[0] - x, goal[1] - y) < WP_TOL:
            wp_i += 1; path = []; replan_now = True
        log.append((t, x, y, th, psi_d, e, u, *pid.scales, D, wp_i, F, Mz, tR, tL, v, w,
                    height(world, x, y), math.degrees(tilt), float(sat)))
        t += DT; step += 1
    if hot:
        leaks_found.append((peak[1], peak[2]))
    # obstacles 'mapped' = those with lidar hits on their surface
    occ_pts = mp.occupied_xy()
    mapped = sum(1 for ox, oy, r in world['obstacles']
                 if len(occ_pts) and np.min(np.hypot(occ_pts[:, 0] - ox, occ_pts[:, 1] - oy)) < r + 0.25)
    L = np.array(log)
    return dict(log=L, time=t, reached=wp_i, leaks=leaks_found, mapped=mapped, n_obs=len(world['obstacles']),
                min_clear=min_clear, path=path_len, max_tilt=math.degrees(max_tilt),
                sat_pct=100.0 * n_sat / max(len(log), 1), peak_torque=float(np.max(np.abs(L[:, 14:16]))),
                mean_abs_e=float(np.mean(np.abs(L[:, 5]))), max_abs_e=float(np.max(np.abs(L[:, 5]))),
                map=mp, collided=collided, success=(wp_i >= len(wps) and not collided))
