# Drone scout extension (added after the guide's question: "why not a drone?")

The ground robot is still the MAIN system. The drone is an extension: a scout that flies ahead over the same hard world.
Files: `python_sim/drone_core.py` (model + controller), `python_sim/compare_drone.py` (run it), `python_sim/drone_result.png`.
Run: `cd python_sim && python compare_drone.py`. 3D view: `cd vpython && python robot_3d_sim.py` (drone flies with the mission; G = gust, C = cycle cameras).

## Model (planar x-z quadrotor)
- Vertical: m z'' = T cos(phi) - m g - bz z' + d_z(t)
- Forward:  m x'' = T sin(phi) - bx x'
- Pitch:    I phi'' = tau - c phi' + d_phi(t)
- Altitude PID gives an acceleration command, thrust T = m (g + a_cmd)/cos(phi) (gravity feed-forward, same idea as the friction feed-forward of the ground robot).
- Outer P loop: speed error -> pitch reference. Inner attitude PID -> torque.
- The SAME fuzzy scheduler (mission_core.fuzzy_scales) adapts Kp, Ki, Kd; the errors are rescaled (0.5 m, 1 m/s for altitude; 0.25 rad, 1.5 rad/s for pitch) onto the ground robot's input range.
- Reference height = terrain height + 1 m (terrain from advanced_core.hard_world, including the steep pile). Gusts: vertical force pulses (-3 N at 4 s, +3 N at 12 s) and a pitch torque pulse at 8 s, each 0.4 s.
- Parameters (ASSUMED, typical small quadrotor, not measured): m = 0.5 kg, I = 0.004 kg m^2, thrust limit 2 x weight, torque limit 0.2 N m, speed 1.5 m/s.

## Results (one run, one layout)
| | Fixed PID | Adaptive fuzzy-PID |
|---|---|---|
| Flight time (24 m) | 20.3 s | 20.3 s |
| RMS altitude error | 0.116 m | 0.090 m (22 percent lower) |
| Peak altitude error (after a gust) | 0.417 m | 0.360 m (14 percent lower) |
| Peak pitch | 22.2 deg | 19.8 deg (take-off transient) |
| Thrust saturation | 0 percent | 0 percent |

Radiation seen from 1 m above the floor (threshold 8, chosen for this layout so the three sources separate): hotspots at x = 9.49, 14.49, 22.97 m, against true source x = 9.4, 14.6, 23.2 m (x error 0.09-0.23 m, along the path only). All 3 sources seen, whereas the ground robot in the advanced scenario found 2/3.

## Ground robot vs drone scout (honest comparison)
| | Ground robot (this project) | Drone scout (this extension) |
|---|---|---|
| Rubble, steep pile, dead end | slips (kappa 0.6), needs A* detour, 74 s | flies over, direct path, about 20 s |
| Endurance | much longer (not modelled here) | minutes; ASSUMED 11.1 Wh at 100 W hover = about 6.7 min, not measured |
| Payload | heavier sensors possible | small payload |
| Radio link inside concrete | cable or better link possible | poor link is a known problem |
| Dynamics modelled | wheels, slip kappa, torque limits | thrust, tilt, gravity, gusts |
| Obstacles | lidar map + A* | NOT modelled (flies at 1 m above terrain; real debris may be taller) |

## Honest limits
Planar (x-z) model, no yaw or roll; assumed parameters; no rotor dynamics, wind model, battery model beyond the one-line estimate, or obstacle avoidance; one run; the detection threshold was tuned on this layout; the drone does not replace the ground robot (it cannot stay long or carry heavy sensors). It shows the controller idea carries over, not that the drone is better.

## What to say to sir
"We kept the ground robot as the main system because the project studies wheeled dynamics and slip. After your question we added a drone scout as an extension: a planar quadrotor model with the same fuzzy-PID scheduler, flying over the same world. It crosses the rubble and the dead end in about 20 s and sees all 3 sources, but it has minutes of battery, a small payload and a poor radio link inside concrete, so the two work together: the drone scouts, the ground robot stays and measures."
