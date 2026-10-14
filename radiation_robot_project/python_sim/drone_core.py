"""Drone-scout extension (planar x-z quadrotor) - same adaptive fuzzy-PID idea as the ground robot.

The ground robot is still the MAIN system. The drone flies ahead over the same hard world
(advanced_core.hard_world: height mounds, U-shaped dead end, hidden sources), holds a fixed
clearance above the terrain and logs radiation. It is a deliberately SIMPLE model:

  vertical :  m z'' = T cos(phi) - m g - bz z' + d_z(t)
  forward  :  m x'' = T sin(phi) - bx x'
  pitch    :  I phi'' = tau - c phi' + d_phi(t)

Controllers (inner attitude loop, altitude loop) are PID with the SAME fuzzy gain scheduler as
the ground robot (mission_core.fuzzy_scales), fed with rescaled errors. An outer P loop turns the
speed error into a pitch reference. Gravity is cancelled by feed-forward T = m (g + a_cmd)/cos(phi).
Assumptions: planar motion, no rotor dynamics, no wind model beyond pulses, perfect state feedback,
no obstacle avoidance (the drone flies above everything at CLEARANCE m), no 3D yaw."""
import math
from mission_core import fuzzy_scales, DT
from advanced_core import hard_world, height

M, I, G = 0.5, 0.004, 9.81            # kg, kg m^2, m/s^2 (small quadrotor, assumed values)
BZ, BX, C = 0.3, 0.3, 0.01            # drag coefficients (assumed)
T_MAX = 2.0 * M * G                   # thrust limit (2:1 thrust-to-weight)
TAU_MAX = 0.2                         # N m pitch torque limit
V_CMD, CLEARANCE = 1.5, 1.0           # forward speed m/s, height above terrain m
PHI_MAX = 0.35                        # rad pitch reference limit
# base gains: altitude (acceleration command) and attitude (angular acceleration command)
KPZ, KIZ, KDZ = 6.0, 1.0, 4.0
KPA, KIA, KDA = 60.0, 0.5, 12.0
# fuzzy normalisation: map metre / rad errors onto the ground robot's (0.6 rad, 1.5 rad/s) scale
EZ_N, DEZ_N = 0.5, 1.0                # m, m/s
EA_N, DEA_N = 0.25, 1.5               # rad, rad/s
# disturbances (time s, vertical force N, pitch torque N m), each lasts 0.4 s
GUSTS = [(4.0, -3.0, 0.0), (8.0, 0.0, 0.03), (12.0, 3.0, -0.03)]
GUST_LEN = 0.4
LEAKS = hard_world()['leaks']
BATT_WH, HOVER_W = 11.1, 100.0       # ASSUMED typical small quadrotor (3S 1000 mAh, ~200 W/kg hover), not measured
S, W, D0 = 45.0, 0.8, 0.1
SCOUT_THRESH = 8.0                    # scouting threshold (lower than the ground robot's 12, drone is farther away)


def dose3d(x, y, h):
    return D0 + sum(S / (1 + (((x - lx) ** 2 + (y - ly) ** 2 + h * h) ** 0.5 / W) ** 2) for lx, ly in LEAKS)


class Loop:
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


def run_drone(adaptive, y0=0.0, x_end=24.0, gusts=GUSTS, t_max=40.0):
    world = hard_world()
    alt, att = Loop(adaptive, (KPZ, KIZ, KDZ), (EZ_N, DEZ_N)), Loop(adaptive, (KPA, KIA, KDA), (EA_N, DEA_N))
    x, z, phi, vx, vz, w, t = 0.0, CLEARANCE, 0.0, 0.0, 0.0, 0.0, 0.0
    log = []
    while x < x_end and t < t_max:
        zref = height(world, x, y0) + CLEARANCE
        ez = zref - z
        a_cmd = max(-5.0, min(8.0, alt.update(ez, DT, 1.0)))
        T = max(0.0, min(T_MAX, M * (G + a_cmd) / max(0.5, math.cos(phi))))
        phi_ref = max(-PHI_MAX, min(PHI_MAX, 0.25 * (V_CMD - vx)))
        ea = phi_ref - phi
        alpha = att.update(ea, DT, 0.5)
        tau = max(-TAU_MAX, min(TAU_MAX, I * alpha + C * w))
        dz = dphi = 0.0
        for g0, fz, tq in gusts:
            if g0 <= t < g0 + GUST_LEN:
                dz, dphi = fz, tq
        ax = (T * math.sin(phi) - BX * vx) / M
        az = (T * math.cos(phi) - M * G - BZ * vz + dz) / M
        aw = (tau - C * w + dphi) / I
        vx += DT * ax; vz += DT * az; w += DT * aw
        x += DT * vx; z += DT * vz; phi += DT * w
        t += DT
        log.append((t, x, z, zref, phi, T, tau, dose3d(x, y0, z - height(world, x, y0)), alt.scales[0], alt.scales[2]))
    return log


def metrics(log):
    n = len(log)
    ez = [r[2] - r[3] for r in log]
    rms = (sum(e * e for e in ez) / n) ** 0.5
    return dict(time=log[-1][0], rms_z=rms, max_z=max(abs(e) for e in ez), max_tilt=max(abs(r[4]) for r in log) * 180 / math.pi,
                peak_T=max(r[5] for r in log), sat_pct=100 * sum(1 for r in log if r[5] >= T_MAX - 1e-9 or r[5] <= 1e-9) / n)


def hotspots(log):
    """dose-weighted centroid (x only) of each run of samples above SCOUT_THRESH"""
    out, run = [], []
    for r in log + [None]:
        if r is not None and r[7] > SCOUT_THRESH:
            run.append(r)
        elif run:
            sw = sum(q[7] for q in run)
            out.append((sum(q[7] * q[1] for q in run) / sw, max(q[7] for q in run)))
            run = []
    return out
