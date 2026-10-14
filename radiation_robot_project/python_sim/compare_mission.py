import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from mission_core import *

F, A = run(False), run(True)
for n, r in (("Fixed PID", F), ("Adaptive fuzzy-PID", A)):
    print(f"{n:20s} time={r['time']:.1f}s reached={r['reached']}/5 leaks={len(r['leaks'])}/3 mapped={r['known']}/10 "
          f"min_clear={r['min_clear']*100:.0f}cm mean|e|={r['mean_abs_e']:.3f}rad max|e|={r['max_abs_e']:.2f}rad")

fig, ax = plt.subplots(3, 1, figsize=(10, 11), gridspec_kw={'height_ratios': [2.2, 1, 1]})
for a, b in SLIP_ZONES: ax[0].axvspan(a, b, color='orange', alpha=.12)
for ox, oy, r in OBSTACLES: ax[0].add_patch(Circle((ox, oy), r, color='0.45'))
for i, (wx, wy) in enumerate(WAYPOINTS):
    ax[0].plot(wx, wy, 'D', color='darkorange'); ax[0].annotate(f"INSPECT {i+1}", (wx, wy), xytext=(0, 9), textcoords='offset points', ha='center', fontsize=8)
for lx, ly in LEAKS: ax[0].plot(lx, ly, 'x', color='red', ms=10, mew=2)
ax[0].plot([r[1] for r in F['log']], [r[2] for r in F['log']], 'r', lw=1.2, label='Fixed PID')
sc = ax[0].scatter([r[1] for r in A['log']][::5], [r[2] for r in A['log']][::5], c=[r[10] for r in A['log']][::5], s=6, cmap='RdYlGn_r', label='Adaptive (colour = radiation)')
for lx, ly in A['leaks']: ax[0].add_patch(Circle((lx, ly), 0.35, fill=False, color='red', lw=1.5))
ax[0].set_aspect('equal'); ax[0].set_xlim(-1, 25); ax[0].set_ylim(-4, 4); ax[0].legend(loc='upper left')
ax[0].set_title('Dynamic route (grey = rubble, x = true leak, circle = leak found by robot)', fontsize=10)
plt.colorbar(sc, ax=ax[0], pad=0.01, label='dose rate')
ax[1].plot([r[0] for r in F['log']], [math.degrees(r[5]) for r in F['log']], 'r', lw=.8, label='Fixed')
ax[1].plot([r[0] for r in A['log']], [math.degrees(r[5]) for r in A['log']], 'g', lw=.8, label='Adaptive')
ax[1].set_ylabel('heading error [deg]'); ax[1].legend()
ax[2].plot([r[0] for r in A['log']], [r[7] for r in A['log']], label='Kp scale'); ax[2].plot([r[0] for r in A['log']], [r[9] for r in A['log']], label='Kd scale')
ax[2].set_ylabel('fuzzy scales'); ax[2].set_xlabel('t [s]'); ax[2].legend()
plt.tight_layout(); plt.savefig('mission_result.png', dpi=100)
