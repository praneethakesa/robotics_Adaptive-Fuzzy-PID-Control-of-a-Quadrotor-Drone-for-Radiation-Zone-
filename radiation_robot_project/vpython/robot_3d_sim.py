"""
Radiation-zone inspection robot - 3D VPython simulation (DYNAMIC ROUTE)
Dynamics & Control of Robotics

There is NO pre-drawn path. The robot is only told the inspection points (orange flags).
It senses obstacles with a limited-range sensor (cyan disc), builds a map as it goes (cyan rings),
and a reactive planner re-computes the desired heading every step (magenta arrow).
The adaptive fuzzy-PID (or fixed PID) steers the robot through rubble slip, debris strikes and
noisy sensing. It also measures radiation (trail colour) and flags leaks (red markers).

Same model + controller as python_sim/mission_core.py and matlab/.
DRONE SCOUT (extension): a small quadrotor takes off with the mission, flies ahead along the route at
1 m height and measures radiation from above (orange rings = sources it saw). Same model as
python_sim/drone_core.py (planar x-z quadrotor, same fuzzy-PID scheduler), here over a flat floor.
The ground robot is still the main system.
Keys (click the canvas first):
  SPACE : debris strike on the ground robot   G : gust on the drone
  F     : toggle Fixed PID / Adaptive fuzzy-PID (both robots)
  C     : cycle camera (ground robot chase / overview / drone chase)   R : reset
Run:  pip install vpython numpy   then   python robot_3d_sim.py
"""
import math, random
from vpython import (canvas, vector, vec, box, cylinder, sphere, curve, label, color,
                     local_light, distant_light, rate, wtext, radians, ring, cone, arrow)

# ============ model (identical to mission_core.py) ============
V_CMD, TAU_V, TAU_W, W_MAX, DT, ROBOT_R = 0.6, 0.20, 0.15, 2.0, 0.01, 0.30
KP0, KI0, KD0 = 2.0, 0.15, 0.25
SLIP_ZONES, SLIP_GAIN = [(6.0, 9.0), (15.0, 18.0)], 0.6
STRIKES, STRIKE_LEN = [(8.0, 1.5), (17.0, -1.5), (26.0, 1.5), (34.0, -1.5)], 0.3
OBSTACLES = [(3.0, 0.6, 0.5), (6.5, -1.6, 0.6), (7.0, 1.8, 0.5), (12.0, 1.0, 0.6), (13.5, -2.5, 0.5),
             (14.5, 2.0, 0.6), (17.5, -0.5, 0.7), (19.5, 2.2, 0.5), (21.0, -2.0, 0.6), (22.5, 1.2, 0.5)]
WAYPOINTS = [(5.0, 0.0), (10.0, -0.6), (15.0, 0.0), (20.0, -0.3), (23.5, -0.5)]
LEAKS = [(10.0, -1.2), (15.5, -1.0), (20.5, 0.8)]
LEAK_S, LEAK_W, DOSE_BG, LEAK_THRESH = 45.0, 0.8, 0.1, 12.0
SENSE_R, WP_TOL, D0, K_REP = 3.0, 0.5, 1.5, 0.45
KP_TAB = [[0.8, 0.9, 1.0], [1.2, 1.3, 1.4], [1.6, 1.8, 2.0]]
KD_TAB = [[0.8, 1.2, 1.6], [0.8, 1.4, 1.8], [0.8, 1.5, 2.0]]
KI_TAB = [1.0, 0.5, 0.0]
E_NORM, DE_NORM = 0.6, 1.5

# ---- drone scout (same values as python_sim/drone_core.py; ASSUMED small quadrotor, not measured) ----
DM, DI, GRAV = 0.5, 0.004, 9.81
DBZ, DBX, DC = 0.3, 0.3, 0.01
T_MAX, TAU_MAX = 2.0 * DM * GRAV, 0.2
D_VCMD, D_CLEAR, PHI_MAX, D_XEND = 1.5, 1.0, 0.35, 24.0
D_ALT, D_ATT = (6.0, 1.0, 4.0), (60.0, 0.5, 12.0)          # base gains: altitude, attitude
D_ALT_N, D_ATT_N = (0.5, 1.0), (0.25, 1.5)                 # fuzzy input rescaling (e, de)
D_GUSTS = [(4.0, -3.0, 0.0), (8.0, 0.0, 0.03), (12.0, 3.0, -0.03)]   # (t, vertical force N, pitch torque N m)
D_GUST_LEN, SCOUT_THRESH = 0.4, 8.0

wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
noise = lambda t: 0.015 * (math.sin(37.0 * t) + math.sin(91.0 * t + 1.0)) / 2.0
slip_at = lambda x: SLIP_GAIN if any(a <= x <= b for a, b in SLIP_ZONES) else 1.0
dose_at = lambda x, y: DOSE_BG + sum(LEAK_S / (1 + (math.hypot(x - a, y - b) / LEAK_W) ** 2) for a, b in LEAKS)


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


class DLoop:
    """PID whose gains are scaled by the SAME fuzzy scheduler as the ground robot (inputs rescaled)."""
    def __init__(self, adaptive, base, norm):
        self.adaptive, self.base, self.norm = adaptive, base, norm
        self.integ = self.e_prev = self.d_f = 0.0
        self.scales = (1.0, 1.0, 1.0)

    def update(self, e, dt, lim_i):
        self.d_f += 0.2 * ((e - self.e_prev) / dt - self.d_f)
        self.e_prev = e
        if self.adaptive:
            self.scales = fuzzy_scales(e * 0.6 / self.norm[0], self.d_f * 1.5 / self.norm[1])
        self.integ = max(-lim_i, min(lim_i, self.integ + e * dt))
        kp, ki, kd = self.base
        return kp * self.scales[0] * e + ki * self.scales[1] * self.integ + kd * self.scales[2] * self.d_f


class Drone:
    def __init__(self, adaptive):
        self.x, self.z, self.phi, self.vx, self.vz, self.w = 0.0, D_CLEAR, 0.0, 0.0, 0.0, 0.0
        self.alt, self.att = DLoop(adaptive, D_ALT, D_ALT_N), DLoop(adaptive, D_ATT, D_ATT_N)
        self.manual_gust_t = -1.0

    def step(self, t, dt, y_line=0.0):
        ez = D_CLEAR - self.z
        a_cmd = max(-5.0, min(8.0, self.alt.update(ez, dt, 1.0)))
        T = max(0.0, min(T_MAX, DM * (GRAV + a_cmd) / max(0.5, math.cos(self.phi))))
        phi_ref = max(-PHI_MAX, min(PHI_MAX, 0.25 * (D_VCMD - self.vx)))
        alpha = self.att.update(phi_ref - self.phi, dt, 0.5)
        tau = max(-TAU_MAX, min(TAU_MAX, DI * alpha + DC * self.w))
        dz = dphi = 0.0
        for g0, fz, tq in D_GUSTS:
            if g0 <= t < g0 + D_GUST_LEN:
                dz, dphi = fz, tq
        if self.manual_gust_t >= 0 and self.manual_gust_t <= t < self.manual_gust_t + D_GUST_LEN:
            dz += -3.0
        self.vx += dt * (T * math.sin(self.phi) - DBX * self.vx) / DM
        self.vz += dt * (T * math.cos(self.phi) - DM * GRAV - DBZ * self.vz + dz) / DM
        self.w += dt * (tau - DC * self.w + dphi) / DI
        self.x += dt * self.vx
        self.z += dt * self.vz
        self.phi += dt * self.w
        return T


def drone_dose(x, y, h):
    return DOSE_BG + sum(LEAK_S / (1 + (math.sqrt((x - a) ** 2 + (y - b) ** 2 + h * h) / LEAK_W) ** 2) for a, b in LEAKS)


class Robot:
    def __init__(self):
        self.x = self.y = self.th = self.v = self.w = 0.0

    def step(self, w_cmd, dt, d_w, slip):
        self.v += dt * (V_CMD * slip - self.v) / TAU_V
        self.w += dt * (w_cmd * slip - self.w) / TAU_W
        self.th += dt * (self.w + d_w)
        self.x += dt * self.v * math.cos(self.th)
        self.y += dt * self.v * math.sin(self.th)


def plan(rb, goal, known):
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


# ============ 3D scene ============
# sim (x, y) -> VPython (x, 0, -y); VPython y is "up".
P = lambda x, y, h=0.0: vector(x, h, -y)
GREY = lambda g: vec(g, g * 0.96, g * 0.9)

scene = canvas(title="<b>Radiation-zone inspection robot</b> - dynamic route, mapping & radiation sensing",
               width=1500, height=780, background=vec(0.02, 0.02, 0.03))
scene.ambient = vec(0.10, 0.10, 0.12)
scene.lights = []
distant_light(direction=vec(-0.3, -1, -0.2), color=vec(0.10, 0.10, 0.13))
random.seed(7)

# --- floor slabs ---
for i in range(-2, 28):
    for zc in (-4.5, -1.5, 1.5, 4.5):
        g = random.uniform(0.28, 0.40)
        tilt = random.uniform(-0.03, 0.03)
        box(pos=vec(i + 0.5, -0.05, zc), size=vec(0.98, 0.1, 2.96), color=GREY(g),
            axis=vec(0.98, 0.98 * tilt, 0), shininess=0.05)

# --- walls, beams, pipes ---
for z in (-6.2, 6.2):
    box(pos=vec(12, 1.8, z), size=vec(32, 3.6, 0.3), color=vec(0.30, 0.30, 0.28))
    for x in range(-2, 27, 4):
        box(pos=vec(x, 1.8, z * 0.97), size=vec(0.35, 3.6, 0.2), color=vec(0.2, 0.2, 0.2))
box(pos=vec(-3.2, 1.8, 0), size=vec(0.3, 3.6, 12.4), color=vec(0.25, 0.25, 0.25))
for x in range(-2, 27, 3):
    box(pos=vec(x, 3.45, 0), size=vec(0.25, 0.25, 12.4), color=vec(0.18, 0.18, 0.2))
for k, (z, r, c) in enumerate(((-5.6, 0.12, vec(0.45, 0.12, 0.10)), (-5.2, 0.08, vec(0.15, 0.30, 0.45)),
                               (5.6, 0.14, vec(0.35, 0.35, 0.38)), (5.2, 0.07, vec(0.5, 0.42, 0.1)))):
    cylinder(pos=vec(-3, 2.9 - 0.1 * k, z), axis=vec(31, 0, 0), radius=r, color=c, shininess=0.4)

# --- hazard stripes + radiation signs ---
for x0 in (2.0, 12.5, 21.5):
    for s in range(10):
        box(pos=vec(x0 + s * 0.2, 0.005, -5.8), size=vec(0.2, 0.01, 0.4),
            color=vec(0.95, 0.8, 0.05) if s % 2 == 0 else vec(0.05, 0.05, 0.05))
for x0 in (3.0, 13.0, 22.0):
    cylinder(pos=vec(x0, 2.0, -6.03), axis=vec(0, 0, 0.03), radius=0.4, color=vec(0.95, 0.8, 0.05))
    cylinder(pos=vec(x0, 2.0, -6.0), axis=vec(0, 0, 0.03), radius=0.09, color=vec(0.05, 0.05, 0.05))
    label(pos=vec(x0, 1.35, -6.0), text="RADIATION AREA", height=9, box=False, line=False,
          color=vec(0.95, 0.8, 0.05), opacity=0)

# --- slip zones: loose gravel/pebbles (the robot can drive over this, but it slips) ---
for a, b in SLIP_ZONES:
    box(pos=P((a + b) / 2, 0.0, 0.005), size=vec(b - a, 0.01, 8.0), color=vec(0.2, 0.2, 0.2), opacity=0.5)
    for _ in range(90):
        x, y, s = random.uniform(a, b), random.uniform(-4, 4), random.uniform(0.03, 0.07)
        box(pos=P(x, y, s * 0.4), size=vec(s * 1.5, s, s * 1.5), color=GREY(random.uniform(0.25, 0.5)))

# --- real obstacles = rubble piles (unknown to the robot until sensed) ---
for ox, oy, r in OBSTACLES:
    box(pos=P(ox, oy, r * 0.55), size=vec(r * 1.3, r * 1.1, r * 1.3), color=GREY(0.3))
    for _ in range(14):
        a, d = random.uniform(0, 2 * math.pi), random.uniform(0, r * 0.85)
        L, s = random.uniform(0.15, 0.4) * r * 2, random.uniform(0.15, 0.45) * r
        box(pos=P(ox + d * math.cos(a), oy + d * math.sin(a), s * 0.5), size=vec(L, s, L * 0.8),
            axis=vec(L, L * random.uniform(-.3, .3), L * random.uniform(-.3, .3)), color=GREY(random.uniform(0.25, 0.5)))

# --- background debris near the walls, puddles, fallen beam ---
for _ in range(25):
    x, y = random.uniform(-2, 26), random.choice((-1, 1)) * random.uniform(4.0, 5.6)
    s = random.uniform(0.1, 0.4)
    box(pos=P(x, y, s * 0.4), size=vec(s * 2, s, s * 1.4), axis=vec(s * 2, s * 2 * random.uniform(-.3, .3), 0),
        color=vec(0.32, 0.3, 0.28))
for x, y in ((10.5, 4.0), (20.0, -4.2)):
    cylinder(pos=P(x, y, 0.005), axis=vec(0, 0.01, 0), radius=0.9, color=vec(0.1, 0.3, 0.35), opacity=0.55, shininess=1)
box(pos=P(13.5, 5.0, 0.35), size=vec(5, 0.3, 0.4), axis=vec(5, 0.25, 1.0), color=vec(0.35, 0.18, 0.12))

# --- inspection waypoints (flags) ---
flags = []
for i, (wx, wy) in enumerate(WAYPOINTS):
    pole = cylinder(pos=P(wx, wy, 0), axis=vec(0, 1.1, 0), radius=0.025, color=vec(0.8, 0.8, 0.8))
    top = sphere(pos=P(wx, wy, 1.15), radius=0.09, color=vec(1.0, 0.55, 0.05), emissive=True)
    label(pos=P(wx, wy, 1.4), text=f"INSPECT {i + 1}", height=10, box=False, line=False,
          color=vec(1.0, 0.7, 0.2), opacity=0)
    flags.append(top)

# --- hidden leaks: shown only after the robot detects them ---
leak_markers = []

# --- robot ---
body = box(pos=vec(0, 0, 0), size=vec(0.62, 0.16, 0.42), color=vec(0.9, 0.55, 0.05))
deck = box(pos=vec(0, 0.11, 0), size=vec(0.45, 0.05, 0.34), color=vec(0.2, 0.2, 0.22))
mast = cylinder(pos=vec(-0.15, 0.13, 0), axis=vec(0, 0.35, 0), radius=0.02, color=vec(0.7, 0.7, 0.7))
cam = box(pos=vec(-0.15, 0.5, 0), size=vec(0.14, 0.09, 0.12), color=vec(0.1, 0.1, 0.12))
lens = cylinder(pos=vec(-0.08, 0.5, 0), axis=vec(0.06, 0, 0), radius=0.03, color=vec(0.2, 0.6, 1.0), emissive=True)
ant = cylinder(pos=vec(0.2, 0.13, 0.12), axis=vec(0, 0.45, 0), radius=0.008, color=vec(1, 1, 1))
ant_tip = sphere(pos=vec(0.2, 0.58, 0.12), radius=0.025, color=vec(1, 0, 0), emissive=True)
sensor = box(pos=vec(0.34, 0.03, 0), size=vec(0.08, 0.1, 0.2), color=vec(0.15, 0.15, 0.2))
wheels = []
for dx in (-0.2, 0.2):
    for dz in (-0.27, 0.27):
        wheels.append(cylinder(pos=vec(dx, -0.03, dz), axis=vec(0, 0, 0.1 if dz > 0 else -0.1),
                               radius=0.1, color=vec(0.05, 0.05, 0.05)))
parts = [body, deck, mast, cam, lens, ant, ant_tip, sensor] + wheels
rel = [(p.pos.x, p.pos.y, p.pos.z, p.axis) for p in parts]
headlamp = local_light(pos=vec(0.4, 0.2, 0), color=vec(1.0, 0.95, 0.8))
beam = cone(pos=vec(0.35, 0.12, 0), axis=vec(1.6, -0.1, 0), radius=0.5, color=vec(1, 0.95, 0.7), opacity=0.06)
parts.append(beam)
rel.append((0.35, 0.12, 0, beam.axis))

# --- drone scout (quadrotor): body, two arms, four rotor discs, camera, blinking lights ---
d_body = box(size=vec(0.34, 0.07, 0.34), color=vec(0.15, 0.2, 0.9))
d_arm1 = box(size=vec(0.62, 0.025, 0.04), color=vec(0.1, 0.1, 0.1))
d_arm2 = box(size=vec(0.04, 0.025, 0.62), color=vec(0.1, 0.1, 0.1))
d_cam = sphere(radius=0.045, color=vec(0.1, 0.8, 1.0), emissive=True)
d_rotors = [cylinder(radius=0.15, color=vec(0.8, 0.85, 1.0), opacity=0.35) for _ in range(4)]
d_light = sphere(radius=0.03, color=vec(0.0, 1.0, 0.0), emissive=True)
D_ROT_POS = [(0.31, 0.03, 0.0), (-0.31, 0.03, 0.0), (0.0, 0.03, 0.31), (0.0, 0.03, -0.31)]
d_label = label(pos=vec(0, 2, 0), text="DRONE SCOUT", height=10, box=False, line=False, color=vec(0.4, 0.8, 1.0), opacity=0)
d_trail = curve(radius=0.02)
d_hot = []


def rot2(rx, ry, phi):
    """rotate a body-frame offset (forward, up) by pitch phi: thrust (up) tilts toward +x for phi > 0"""
    return rx * math.cos(phi) + ry * math.sin(phi), -rx * math.sin(phi) + ry * math.cos(phi)


def place_drone(dr):
    ph = dr.phi
    base = vec(dr.x, dr.z, 0.0)
    fwd, up = vec(math.cos(ph), -math.sin(ph), 0), vec(math.sin(ph), math.cos(ph), 0)
    for part in (d_body, d_arm1, d_arm2):
        part.pos = base
        part.axis = fwd * part.size.x
        part.up = up
    ox, oy = rot2(0.0, -0.07, ph)
    d_cam.pos = base + vec(ox, oy, 0)
    for r, (rx, ry, rz) in zip(d_rotors, D_ROT_POS):
        ox, oy = rot2(rx, ry, ph)
        r.pos = base + vec(ox, oy, rz)
        r.axis = up * 0.012
    ox, oy = rot2(0.17, 0.05, ph)
    d_light.pos = base + vec(ox, oy, 0)
    d_label.pos = base + vec(0, 0.6, 0)


d_marks = [d_body, d_arm1, d_arm2, d_cam, d_light, d_label] + d_rotors

sense_disc = cylinder(pos=P(0, 0, 0.01), axis=vec(0, 0.005, 0), radius=SENSE_R, color=vec(0.1, 0.8, 0.9), opacity=0.07)
plan_arrow = arrow(pos=P(0, 0, 0.3), axis=vec(1.2, 0, 0), shaftwidth=0.05, color=vec(1.0, 0.2, 0.9))
trail = curve(radius=0.03)
map_rings = []

status = wtext(text="")
scene.append_to_caption("\n")


class State:
    pass


S = State()
S.adaptive, S.cam = True, 0


def reset():
    S.rb, S.pid, S.t = Robot(), PID(S.adaptive), 0.0
    S.known, S.leaks, S.wp = set(), [], 0
    S.hot, S.peak = False, (0.0, 0.0, 0.0)
    S.strike_t, S.strike_dir = -1.0, 1.0
    S.min_clear, S.done = 99.0, False
    S.dr, S.d_done, S.d_hot, S.d_peak, S.d_found = Drone(S.adaptive), False, False, (0.0, 0.0), []
    place_drone(S.dr)
    trail.clear()
    d_trail.clear()
    for m in map_rings + leak_markers + d_hot:
        m.visible = False
    del map_rings[:], leak_markers[:], d_hot[:]
    for f in flags:
        f.color = vec(1.0, 0.55, 0.05)


def on_key(evt):
    k = evt.key
    if k == " ":
        S.strike_t, S.strike_dir = S.t, -S.strike_dir
    elif k in ("f", "F"):
        S.adaptive = not S.adaptive
        reset()
    elif k in ("g", "G"):
        S.dr.manual_gust_t = S.t
    elif k in ("c", "C"):
        S.cam = (S.cam + 1) % 3
    elif k in ("r", "R"):
        reset()


scene.bind("keydown", on_key)


def place(rb, bob):
    c, s = math.cos(rb.th), math.sin(rb.th)
    fwd, right = vec(c, 0, -s), vec(s, 0, c)
    base = P(rb.x, rb.y, 0.13 + bob)
    for p, (rx, ry, rz, ax) in zip(parts, rel):
        p.pos = base + fwd * rx + vec(0, ry, 0) + right * rz
        if p in wheels:
            p.axis = right * (0.1 if rz > 0 else -0.1)
        elif p is beam:
            p.axis = fwd * 1.6 + vec(0, -0.1, 0)
        elif p in (body, deck, cam, sensor):
            p.axis = fwd * p.size.x
            p.up = vec(0, 1, 0)
        elif p is lens:
            p.axis = fwd * 0.06
        else:
            p.axis = vec(0, ax.y, 0)
    headlamp.pos = base + fwd * 0.5 + vec(0, 0.25, 0)


def camera():
    rb = S.rb
    if S.cam == 2:
        scene.camera.pos = vec(S.dr.x - 2.6, S.dr.z + 1.2, 2.8)
        scene.camera.axis = vec(S.dr.x + 1.0, S.dr.z - 0.2, 0.0) - scene.camera.pos
    elif S.cam == 0:
        c, s = math.cos(rb.th), math.sin(rb.th)
        scene.camera.pos = P(rb.x, rb.y, 0) + vec(-3.4 * c, 1.9, 3.4 * s)
        scene.camera.axis = P(rb.x + 1.5 * c, rb.y + 1.5 * s, 0.2) - scene.camera.pos
    else:
        scene.camera.pos = vec(12, 15, 5)
        scene.camera.axis = vec(0, -15, -5)


def dose_color(D):
    f = min(D / 30.0, 1.0)
    return vec(f, 1.0 - 0.75 * f, 0.15)


def commit_leak(pk):
    S.leaks.append((pk[1], pk[2]))
    n = len(S.leaks)
    m1 = sphere(pos=P(pk[1], pk[2], 0.18), radius=0.16, color=vec(1, 0.1, 0.05), emissive=True)
    m2 = label(pos=P(pk[1], pk[2], 0.8), text=f"LEAK {n}  (peak {pk[0]:.0f})", height=12, box=False,
               line=False, color=vec(1, 0.35, 0.25), opacity=0)
    leak_markers.extend([m1, m2])


# ============ main loop ============
reset()
frame = 0
while True:
    rate(100)
    rb, t = S.rb, S.t
    if not S.d_done:
        S.dr.step(S.t, DT)
        dd = drone_dose(S.dr.x, 0.0, S.dr.z)
        if dd > SCOUT_THRESH:
            if not S.d_hot or dd > S.d_peak[0]:
                S.d_peak = (dd, S.dr.x)
            S.d_hot = True
        elif S.d_hot:
            S.d_found.append(S.d_peak)
            S.d_hot = False
            d_hot.append(ring(pos=vec(S.d_peak[1], 0.04, 0.0), axis=vec(0, 1, 0), radius=0.5, thickness=0.05,
                              color=vec(1.0, 0.6, 0.1)))
        place_drone(S.dr)
        if frame % 3 == 0:
            d_trail.append(pos=vec(S.dr.x, S.dr.z, 0.0), color=vec(0.4, 0.8, 1.0))
            if S.cam == 2:
                camera()
        d_light.color = vec(0.0, 1.0, 0.0) if int(S.t * 4) % 2 == 0 else vec(0.0, 0.3, 0.0)
        if S.dr.x >= D_XEND:
            if S.d_hot:
                S.d_found.append(S.d_peak)
                d_hot.append(ring(pos=vec(S.d_peak[1], 0.04, 0.0), axis=vec(0, 1, 0), radius=0.5, thickness=0.05,
                                  color=vec(1.0, 0.6, 0.1)))
            S.d_done = True
            d_label.text = f"DRONE SCOUT: done, {len(S.d_found)} sources seen"
    if S.done:
        frame += 1
        continue
    new = [i for i, (ox, oy, r) in enumerate(OBSTACLES)
           if i not in S.known and math.hypot(rb.x - ox, rb.y - oy) - r < SENSE_R]
    for i in new:
        S.known.add(i)
        ox, oy, r = OBSTACLES[i]
        map_rings.append(ring(pos=P(ox, oy, 0.03), axis=vec(0, 1, 0), radius=r + 0.08, thickness=0.05,
                              color=vec(0.1, 0.9, 0.95)))
    goal = WAYPOINTS[S.wp]
    psi_d = plan(rb, goal, S.known)
    e = wrap(psi_d - (rb.th + noise(t)))
    u = S.pid.update(e, DT)
    d_w = 0.0
    for ts, a in STRIKES:
        if ts <= t < ts + STRIKE_LEN:
            d_w = a
    if S.strike_t >= 0 and S.strike_t <= t < S.strike_t + STRIKE_LEN:
        d_w += 1.5 * S.strike_dir
    sl = slip_at(rb.x)
    rb.step(u, DT, d_w, sl)
    S.t += DT
    D = dose_at(rb.x, rb.y)
    if D > LEAK_THRESH:
        if not S.hot or D > S.peak[0]:
            S.peak = (D, rb.x, rb.y)
        S.hot = True
    elif S.hot:
        commit_leak(S.peak)
        S.hot = False
    clear = min(math.hypot(rb.x - ox, rb.y - oy) - r - ROBOT_R for ox, oy, r in OBSTACLES)
    S.min_clear = min(S.min_clear, clear)
    if math.hypot(goal[0] - rb.x, goal[1] - rb.y) < WP_TOL:
        flags[S.wp].color = vec(0.2, 1.0, 0.2)
        S.wp += 1
        if S.wp >= len(WAYPOINTS):
            S.done = True
            if S.hot:
                commit_leak(S.peak)
            lk = ", ".join(f"({a:.1f}, {b:.1f})" for a, b in S.leaks)
            status.text = (f"<b>MISSION COMPLETE</b> ({'Adaptive fuzzy-PID' if S.adaptive else 'Fixed PID'}) in {S.t:.1f} s | "
                           f"inspection points 5/5 | obstacles mapped {len(S.known)}/{len(OBSTACLES)} | "
                           f"leaks found {len(S.leaks)}/{len(LEAKS)} at {lk} | min clearance {S.min_clear*100:.0f} cm "
                           f"&nbsp;&nbsp;(R = rerun, F = switch controller)")
            continue
    place(rb, 0.012 * math.sin(40 * S.t) * (1 if sl < 1 else 0.2) + (0.02 if d_w else 0.0))
    sense_disc.pos = P(rb.x, rb.y, 0.01)
    plan_arrow.pos = P(rb.x, rb.y, 0.3)
    plan_arrow.axis = vec(math.cos(psi_d), 0, -math.sin(psi_d)) * 1.2
    frame += 1
    if frame % 3 == 0:
        trail.append(pos=P(rb.x, rb.y, 0.03), color=dose_color(D))
        camera()
    ant_tip.color = vec(1, 0, 0) if int(S.t * 2) % 2 == 0 else vec(0.3, 0, 0)
    if frame % 10 == 0:
        kp, ki, kd = S.pid.scales
        zone = "RUBBLE (slip)" if sl < 1 else "clear floor"
        hit = " <b style='color:red'>DEBRIS STRIKE!</b>" if d_w else ""
        c = dose_color(D)
        hexc = "#%02x%02x%02x" % (int(c.x * 255), int(c.y * 255), int(c.z * 255))
        status.text = (f"<b>{'Adaptive fuzzy-PID' if S.adaptive else 'Fixed PID'}</b> | t={S.t:5.1f}s | "
                       f"going to INSPECT {S.wp + 1}/5 | mapped {len(S.known)}/{len(OBSTACLES)} obstacles | "
                       f"leaks found {len(S.leaks)} | radiation <b style='color:{hexc}'>{D:5.1f}</b> | "
                       f"nearest obstacle {clear*100:4.0f} cm | heading err {math.degrees(e):5.1f} deg | "
                       f"Kp x{kp:.2f} Kd x{kd:.2f} Ki x{ki:.2f} | {zone}{hit}<br>"
                       f"<b>Drone scout</b> | x={S.dr.x:4.1f} m | height {S.dr.z:4.2f} m | pitch {math.degrees(S.dr.phi):4.1f} deg | "
                       f"sources seen {len(S.d_found)}/{len(LEAKS)} | {'finished' if S.d_done else 'flying'} (G = gust)")
