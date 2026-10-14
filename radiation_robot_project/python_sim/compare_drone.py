"""Drone scout: fixed PID vs adaptive fuzzy-PID, terrain following over the hard world. Run: python compare_drone.py"""
import math
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from drone_core import *

F, A = run_drone(False), run_drone(True)
print(f"{'controller':20s}{'time s':>8}{'rms z err m':>13}{'max z err m':>13}{'max tilt deg':>14}{'peak T N':>10}{'sat %':>7}")
for n, lg in (("Fixed PID", F), ("Adaptive fuzzy-PID", A)):
    m = metrics(lg)
    print(f"{n:20s}{m['time']:8.1f}{m['rms_z']:13.3f}{m['max_z']:13.3f}{m['max_tilt']:14.1f}{m['peak_T']:10.2f}{m['sat_pct']:7.1f}")
print("hotspots seen by drone (x position [m], peak dose), true sources at x =", [l[0] for l in LEAKS])
for x, d in hotspots(A):
    print(f"  x_hat={x:.2f}  peak={d:.1f}  error to nearest source x={min(abs(x - l[0]) for l in LEAKS):.2f} m")
print(f"battery (ASSUMED, not measured): {BATT_WH} Wh at {HOVER_W} W hover -> about {BATT_WH/HOVER_W*60:.1f} min")

fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
for lg, c, n in ((F, 'r', 'Fixed PID'), (A, 'g', 'Adaptive fuzzy-PID')):
    ax[1].plot([r[1] for r in lg], [r[2] - r[3] for r in lg], c, lw=.9, label=n)
    ax[2].plot([r[1] for r in lg], [math.degrees(r[4]) for r in lg], c, lw=.9, label=n)
ax[0].plot([r[1] for r in A], [r[3] for r in A], 'k--', lw=1, label='reference (terrain + 1 m)')
ax[0].plot([r[1] for r in A], [r[2] for r in A], 'g', lw=1.2, label='drone altitude (adaptive)')
ax[0].fill_between([r[1] for r in A], 0, [r[3] - CLEARANCE for r in A], color='0.7', label='terrain')
ax[0].set_ylabel('height z [m]'); ax[0].legend(loc='upper right', fontsize=8)
for lx, ly in LEAKS: ax[0].axvline(lx, color='red', ls=':', lw=1)
ax[1].set_ylabel('altitude error [m]'); ax[1].legend(fontsize=8)
ax[2].set_ylabel('pitch [deg]'); ax[2].legend(fontsize=8)
ax[3].plot([r[1] for r in A], [r[7] for r in A], 'darkorange'); ax[3].axhline(SCOUT_THRESH, color='k', ls='--', lw=.8)
ax[3].set_ylabel('dose rate seen'); ax[3].set_xlabel('x along flight [m]  (red dotted = true sources)')
for a in ax[:3]:
    for g0, _, _ in GUSTS: a.axvline(min(r[1] for r in A if r[0] >= g0), color='b', alpha=.3)
ax[0].set_title('Drone scout over the hard world (blue bands = gust disturbances)', fontsize=10)
plt.tight_layout(); plt.savefig('drone_result.png', dpi=100)
