"""Dynamic-route inspection mission (no pre-drawn path).

The robot is given only INSPECTION POINTS. It senses obstacles with a limited-range
sensor, builds a map as it goes, and a reactive planner re-computes the desired heading
every step. The (adaptive fuzzy-PID | fixed PID) heading controller then steers the robot
through rubble slip, debris strikes and noisy sensing. It also measures radiation and
flags leaks. MATLAB (../matlab) implements exactly the same logic."""
import math

# ---------- robot / actuator ----------
V_CMD, TAU_V, TAU_W, W_MAX, DT = 0.6, 0.20, 0.15, 2.0, 0.01
ROBOT_R = 0.30
KP0, KI0, KD0 = 2.0, 0.15, 0.25
SLIP_ZONES, SLIP_GAIN = [(6.0, 9.0), (15.0, 18.0)], 0.6
STRIKES, STRIKE_LEN = [(8.0, 1.5), (17.0, -1.5), (26.0, 1.5), (34.0, -1.5)], 0.3

# ---------- world (unknown to the robot until sensed) ----------
OBSTACLES = [(3.0, 0.6, 0.5), (6.5, -1.6, 0.6), (7.0, 1.8, 0.5), (12.0, 1.0, 0.6), (13.5, -2.5, 0.5),
             (14.5, 2.0, 0.6), (17.5, -0.5, 0.7), (19.5, 2.2, 0.5), (21.0, -2.0, 0.6), (22.5, 1.2, 0.5)]
WAYPOINTS = [(5.0, 0.0), (10.0, -0.6), (15.0, 0.0), (20.0, -0.3), (23.5, -0.5)]
LEAKS = [(10.0, -1.2), (15.5, -1.0), (20.5, 0.8)]   # hidden radiation sources
LEAK_S, LEAK_W, DOSE_BG, LEAK_THRESH = 45.0, 0.8, 0.1, 12.0
SENSE_R, WP_TOL = 3.0, 0.5
D0, K_REP = 1.5, 0.45          # planner: influence distance, repulsion gain


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def noise(t):
    return 0.015 * (math.sin(37.0 * t) + math.sin(91.0 * t + 1.0)) / 2.0


def slip_at(x):
    return SLIP_GAIN if any(a <= x <= b for a, b in SLIP_ZONES) else 1.0


def strike_at(t, strikes=STRIKES):
    for ts, a in strikes:
        if ts <= t < ts + STRIKE_LEN:
            return a
    return 0.0


def dose_at(x, y):
    return DOSE_BG + sum(LEAK_S / (1 + (math.hypot(x - lx, y - ly) / LEAK_W) ** 2) for lx, ly in LEAKS)


# ---------- fuzzy-PID ----------
KP_TAB = [[0.8, 0.9, 1.0], [1.2, 1.3, 1.4], [1.6, 1.8, 2.0]]
KD_TAB = [[0.8, 1.2, 1.6], [0.8, 1.4, 1.8], [0.8, 1.5, 2.0]]
KI_TAB = [1.0, 0.5, 0.0]
E_NORM, DE_NORM = 0.6, 1.5


def memberships(u):
    u = min(max(u, 0.0), 1.0)
    return (max(0.0, 1 - 2 * u), max(0.0, 1 - 2 * abs(u - 0.5)), max(0.0, 2 * u - 1))


def fuzzy_scales(e, de):
    me, md = memberships(abs(e) / E_NORM), memberships(abs(de) / DE_NORM)
    skp = sum(me[i] * md[j] * KP_TAB[i][j] for i in range(3) for j in range(3))
    skd = sum(me[i] * md[j] * KD_TAB[i][j] for i in range(3) for j in range(3))
    return skp, sum(me[i] * KI_TAB[i] for i in range(3)), skd


class PID:
    def __init__(self, adaptive):
        self.adaptive, self.integ, self.e_prev, self.d_f, self.scales = adaptive, 0.0, 0.0, 0.0, (1.0, 1.0, 1.0)

    def update(self, e, dt):
        self.d_f += 0.2 * ((e - self.e_prev) / dt - self.d_f)
        self.e_prev = e
        self.scales = fuzzy_scales(e, self.d_f) if self.adaptive else (1.0, 1.0, 1.0)
        self.integ = max(-0.5, min(0.5, self.integ + e * dt))
        u = KP0 * self.scales[0] * e + KI0 * self.scales[1] * self.integ + KD0 * self.scales[2] * self.d_f
        return max(-W_MAX, min(W_MAX, u))


class Robot:
    def __init__(self):
        self.x = self.y = self.th = self.v = self.w = 0.0

    def step(self, w_cmd, dt, d_w, slip):
        self.v += dt * (V_CMD * slip - self.v) / TAU_V
        self.w += dt * (w_cmd * slip - self.w) / TAU_W
        self.th += dt * (self.w + d_w)
        self.x += dt * self.v * math.cos(self.th)
        self.y += dt * self.v * math.sin(self.th)


# ---------- sensing + dynamic planner ----------
def sense(rb, known):
    """Limited-range sensor: obstacles whose surface is within SENSE_R get added to the map."""
    new = []
    for i, (ox, oy, r) in enumerate(OBSTACLES):
        if i not in known and math.hypot(rb.x - ox, rb.y - oy) - r < SENSE_R:
            known.add(i)
            new.append(i)
    return new


def plan(rb, goal, known):
    """Reactive planner: goal attraction + repulsion/slide-around from the MAPPED obstacles only."""
    dx, dy = goal[0] - rb.x, goal[1] - rb.y
    dist = math.hypot(dx, dy)
    ux, uy = dx / dist, dy / dist
    fx, fy = ux, uy
    for i in known:
        ox, oy, r = OBSTACLES[i]
        ex, ey = rb.x - ox, rb.y - oy
        c = math.hypot(ex, ey)
        d = c - r - ROBOT_R
        if d < D0:
            nx, ny = ex / c, ey / c
            tx, ty = -ny, nx
            if tx * ux + ty * uy < 0:
                tx, ty = -tx, -ty
            mag = K_REP * (1.0 / max(d, 0.05) - 1.0 / D0)
            fx += mag * (0.6 * nx + 0.8 * tx)
            fy += mag * (0.6 * ny + 0.8 * ty)
    return math.atan2(fy, fx)


def run(adaptive, strikes=STRIKES, t_max=120.0):
    rb, pid = Robot(), PID(adaptive)
    known, leaks_found, wp_i, t = set(), [], 0, 0.0
    hot, peak = False, (0.0, 0.0, 0.0)   # radiation 'hot episode' tracker
    log, min_clear, path_len = [], 99.0, 0.0
    while wp_i < len(WAYPOINTS) and t < t_max:
        sense(rb, known)
        goal = WAYPOINTS[wp_i]
        psi_d = plan(rb, goal, known)
        e = wrap(psi_d - (rb.th + noise(t)))
        u = pid.update(e, DT)
        px, py = rb.x, rb.y
        rb.step(u, DT, strike_at(t, strikes), slip_at(rb.x))
        path_len += math.hypot(rb.x - px, rb.y - py)
        D = dose_at(rb.x, rb.y)
        if D > LEAK_THRESH:                      # inside a hot zone: remember where the reading peaks
            if not hot or D > peak[0]:
                peak = (D, rb.x, rb.y)
            hot = True
        elif hot:                                # left the zone: commit the peak as a leak location
            leaks_found.append((peak[1], peak[2]))
            hot = False
        min_clear = min(min_clear, min(math.hypot(rb.x - ox, rb.y - oy) - r - ROBOT_R for ox, oy, r in OBSTACLES))
        if math.hypot(goal[0] - rb.x, goal[1] - rb.y) < WP_TOL:
            wp_i += 1
        log.append((t, rb.x, rb.y, rb.th, psi_d, e, u, *pid.scales, D, wp_i))
        t += DT
    if hot:
        leaks_found.append((peak[1], peak[2]))
    return dict(log=log, time=t, reached=wp_i, leaks=leaks_found, known=len(known),
                min_clear=min_clear, path=path_len,
                mean_abs_e=sum(abs(r[5]) for r in log) / len(log), max_abs_e=max(abs(r[5]) for r in log))
