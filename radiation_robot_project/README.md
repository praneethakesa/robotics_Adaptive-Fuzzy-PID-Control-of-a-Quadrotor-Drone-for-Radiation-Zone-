# Dynamic-Route Inspection Robot for a Radiation Zone
Dynamics and Control of Robotics - adaptive fuzzy-PID heading control

## Folder map
| Folder | What it is |
|---|---|
| report/model_and_control.pdf | The equations: kinematics, Newton-Euler and Lagrange dynamics, servo + slip model, planner, fuzzy-PID law, linearisation, Routh-Hurwitz, poles and margins, data identification. Source in model_and_control.tex |
| matlab/ | All the MATLAB work. Open this folder in MATLAB and run `run_all` |
| matlab/data/ | Two CSV files (SYNTHETIC, see below) used by the data scripts |
| docs/ | Viva Q&A on terrain, limitations and future work |
| vpython/ | 3D visual simulation (`python robot_3d_sim.py`) |
| python_sim/ | Python reference implementation (`python compare_mission.py`, advanced: `python compare_advanced.py`) |

## MATLAB
- `run_all.m` runs everything in order
- `analyze_stability.m` G(s), C(s), characteristic polynomial, Routh-Hurwitz, poles, Bode, step, root locus (needs Control System Toolbox)
- `identify_actuator.m` fits the servo time constant and the slip ratio from a CSV (data connection 1)
- `analyze_radiation.m` estimates hotspots from a radiation log CSV (data connection 2)
- `run_mission.m` / `simulate_mission.m` nonlinear dynamic-route mission, fixed PID vs adaptive fuzzy-PID, plus F, M and wheel-torque plots
- `robot_params.m`, `robot_accel.m`, `fuzzy_scales.m` parameters, dynamics, fuzzy scheduler
- `compare_advanced.m` ADVANCED scenario (see below): runs 8 combinations and prints a table + figures
- `adv_params.m`, `world_hard.m`, `simulate_advanced.m` parameters, world (25 obstacles, height mounds, dead end) and the advanced simulation
- `compare_pid.m` runs fixed PID and adaptive fuzzy-PID on the same mission, prints a metrics table (with % change) and 4 comparison figures
- `demo_key_values.m` prints fuzzy scales and F, M, wheel torques for clear floor vs rubble

Note: `fuzzy_scales`, `robot_accel` and `simulate_mission` are functions and need arguments, e.g. `[kp,ki,kd] = fuzzy_scales(0.5,1.0)`. Typing the bare name gives "Not enough input arguments"; that is normal.

## About the datasets (read this)
`actuator_step_log_SYNTHETIC.csv` and `radiation_log_SYNTHETIC.csv` were generated from the model plus noise, so the pipeline runs out of the box. They are NOT real measurements. To connect real data, log the same columns from your robot or from a simulator (for example Webots) and replace the file. No other change is needed.

## Results (Python reference run of the nonlinear simulation)
| | Fixed PID | Adaptive fuzzy-PID |
|---|---|---|
| Mission time | 47.4 s | 47.3 s |
| Points reached / leaks found / obstacles mapped | 5/5, 3/3, 10/10 | 5/5, 3/3, 10/10 |
| Mean heading error | 0.107 rad | 0.099 rad (7 percent lower) |
| Peak heading error | 0.91 rad | 0.87 rad (4 percent lower) |
| Peak wheel torque | 2.49 N m | 2.49 N m |

Stability analysis (frozen-gain): slip cuts the loop crossover from 1.99 to 1.19 rad/s; the adaptive gains restore it to 2.14 rad/s with phase margin about 84 degrees. This is not a proof of stability of the time-varying fuzzy law.

## Verification status
- Python reference and the F/M/torque form of the dynamics give identical mission times (47.45 s and 47.35 s).
- The fitting and hotspot logic were checked in Python on the same CSVs: kappa 1.00 / 0.60, tau about 0.15 s, three hotspots found with 0.7-0.9 m error.
- `run_all` has been run in MATLAB: all four stability cases and 16 corners stable, kappa 1.001 / 0.600, tau 0.152 / 0.147 s, mission results identical to Python, hotspot errors 0.68-0.94 m.
- `demo_key_values.m`, `compare_pid.m` and `compare_advanced.m` have not been run in MATLAB itself by the author; run them and report any error. The numeric parts of `simulate_advanced.m` and `compare_advanced.m` were run in GNU Octave 8.4 and matched the Python reference for all 8 scenarios; the original `simulate_mission` also reproduced your MATLAB numbers in Octave. The plotting code could not be tested (no graphics in the test environment).

## Advanced extension (implemented, not just proposed)
The planar mission above is unchanged. A second scenario removes the main limitations:

| Limitation | What was added | Where |
|---|---|---|
| Flat floor | Height map z(x,y); slope along the heading adds a gravity term m g sin(phi); tilt logged; slope lowers traction | `simulate_advanced.m`, `advanced_core.py` |
| No torque limits | Wheel torques saturate (3 N m wheeled, 4 N m tracked), then F and M are recomputed | same |
| Known obstacle list | 2D lidar (90 rays, 4 m, 10 Hz) builds an occupancy grid | same |
| Reactive planner can be trapped | A* on the inflated grid (global) + reactive layer (local); U-shaped dead end in the world | same |
| Wheeled only | `wheeled` / `tracked` drive types (slip, climbable slope, torque limit) | `adv_params.m` |

Run `compare_advanced` in MATLAB or `python compare_advanced.py` in `python_sim/`.

Results (same in MATLAB/Octave and Python, one fixed layout):

| Drive | Planner | Outcome | Time (fixed / adaptive) | Mean heading error (fixed / adaptive) |
|---|---|---|---|---|
| wheeled | reactive-only | hits the dead end, 3/4 points | 45 / 46 s | 0.384 / 0.385 rad |
| wheeled | A* + reactive | 4/4 points, 29-31 cm clearance | 74.4 / 74.5 s | 0.217 / 0.210 rad |
| tracked | reactive-only | hits the dead end, 3/4 points | 39 s | 0.151 / 0.135 rad |
| tracked | A* + reactive | 4/4 points, 39 cm clearance | 64.2 / 64.5 s | 0.194 / 0.183 rad |

Honest limits of the extension: one layout, not a statistical study; 2 of 3 leaks found (the replanned route passes 1.56 m from one source, beyond the 1.3 m detection range); tilt is reported but there is no tip-over dynamics; getting stuck is only represented by the slope and torque limits; the lidar map is binary, static and assumes known pose (not full SLAM); `tracked` is a parameter change, not a track-terrain contact model; terrain slope is assumed sensed within lidar range; the 3D VPython view was not updated.

## Drone scout extension (added later)
`python_sim/drone_core.py`, `python_sim/compare_drone.py` (run `python compare_drone.py`), result `python_sim/drone_result.png`, notes and viva answer in `docs/drone_scout_extension.md`. A planar quadrotor flies over the hard world with the same fuzzy-PID scheduler: RMS altitude error 0.116 m (fixed) vs 0.090 m (adaptive), 3 of 3 sources seen. The 3D VPython view (`vpython/robot_3d_sim.py`) now also shows the drone: blue quadrotor flying 1 m above the floor, orange rings where it sees a source, key G = gust, key C cycles cameras (ground robot / overview / drone). In the VPython scene the floor is flat (the planar mission world). Python and VPython only (no MATLAB version); parameters are assumed, one run. The VPython drone logic was tested headless with a stub (drone reached 24 m, 3/3 sources seen at x = 10.1, 15.7, 20.5 m); the actual 3D window was not opened here, so run it and report any error.

## Real or Webots data
No real data is included and I cannot create any. The two CSV scripts accept real logs in the same columns (`t, u_cmd, omega_meas, surface` and `t, x, y, dose_rate`); replace the files in `matlab/data/` and run `identify_actuator` and `analyze_radiation`.
Prepared answers for terrain and limitation questions: `docs/viva_questions_terrain_and_limits.md`.
