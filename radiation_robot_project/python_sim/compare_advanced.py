"""Advanced scenario: dense obstacles + 3D terrain + dead end. Compares reactive-only vs A*+reactive,
fixed PID vs adaptive fuzzy-PID, wheeled vs tracked. Writes advanced_result.png."""
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from advanced_core import *

W = hard_world()
runs = {}
print(f"{'drive':8s} {'planner':14s} {'controller':10s} {'outcome':10s} {'time':>6s} {'pts':>4s} {'leaks':>5s} "
      f"{'clear':>6s} {'tilt':>5s} {'sat%':>5s} {'peakT':>6s} {'mean|e|':>8s}")
for drive in ('wheeled', 'tracked'):
    for ug in (False, True):
        for ad in (False, True):
            r = run_adv(W, ad, ug, drive)
            runs[(drive, ug, ad)] = r
            outcome = 'OK' if r['success'] else ('COLLISION' if r['collided'] else 'TIMEOUT')
            print(f"{drive:8s} {'A*+reactive' if ug else 'reactive-only':14s} {'adaptive' if ad else 'fixed':10s} {outcome:10s} "
                  f"{r['time']:6.1f} {r['reached']:>2d}/4 {len(r['leaks']):>3d}/3 {r['min_clear']*100:5.0f}c "
                  f"{r['max_tilt']:5.1f} {r['sat_pct']:5.1f} {r['peak_torque']:6.2f} {r['mean_abs_e']:8.3f}")

fig, ax = plt.subplots(4, 1, figsize=(12, 15), gridspec_kw={'height_ratios': [2, 2, 2, 1.4]})
sl = slope_grid(W)
for a, key, title in ((ax[0], ('wheeled', False, True), 'Wheeled, reactive-only: runs into the dead end'),
                      (ax[1], ('wheeled', True, True), 'Wheeled, A* + reactive: detects the dead end, plans around, avoids the steep pile'),
                      (ax[2], ('tracked', True, True), 'Tracked, A* + reactive: higher slope limit and less slip on rubble, shorter faster run')):
    r = runs[key]
    a.contourf(CXX, CYY, sl, levels=[0.2, 0.45, 1], colors=['#ffd0d0', '#ff9090'], alpha=.7)
    for ox, oy, rr in W['obstacles']: a.add_patch(Circle((ox, oy), rr, color='0.4'))
    a.plot(r['log'][:, 1], r['log'][:, 2], 'g', lw=1.2)
    for wx, wy in W['waypoints']: a.plot(wx, wy, 'D', c='darkorange')
    for lx, ly in W['leaks']: a.plot(lx, ly, 'rx', ms=8, mew=2)
    a.set_aspect('equal'); a.set_xlim(-1, 27); a.set_ylim(-5, 5); a.set_title(title, fontsize=10)
for k, c in ((('wheeled', True, False), 'r'), (('wheeled', True, True), 'g')):
    L = runs[k]['log']; ax[3].plot(L[:, 0], L[:, 14], c, lw=.6); ax[3].plot(L[:, 0], L[:, 15], c, lw=.6, alpha=.5)
ax[3].axhline(DRIVES['wheeled']['tau_max'], ls='--', c='k'); ax[3].axhline(-DRIVES['wheeled']['tau_max'], ls='--', c='k')
ax[3].set_ylabel('wheel torque [N m]'); ax[3].set_xlabel('t [s]'); ax[3].set_title('Wheel torques (dashed = motor limit); red = fixed, green = adaptive')
plt.tight_layout(); plt.savefig('advanced_result.png', dpi=90)
