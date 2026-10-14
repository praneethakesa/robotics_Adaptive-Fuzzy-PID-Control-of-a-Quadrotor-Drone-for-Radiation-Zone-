# Viva Q&A: real accident sites, terrain and limitations

## Opening statement
Sir, a real accident site is not a flat plane. My project models the **planar case**: flat ground with many
obstacles and slippery patches. I chose it because the course is about dynamics and control, and the planar model
lets me derive the equations exactly and analyse the controller. Real terrain is a limitation, and I know how it
would be extended.

## What the model already covers
| Real-site problem | How the project represents it |
|---|---|
| Many obstacles | 10 obstacles, unknown at the start; a 3 m sensor finds them and the robot maps them as it moves |
| No map after an accident | No pre-drawn path; the planner re-plans the heading every 0.01 s from the obstacles seen so far |
| Loose rubble, wheels slip | Slip ratio kappa = 0.6 on rubble; the adaptive controller compensates |
| Falling debris | Yaw-rate kicks of +/-1.5 rad/s, 4 times |
| Noisy sensing | Heading sensor noise |

## Extension that is now in the project (advanced scenario)
Height-map terrain with a gravity term, motor torque limits, lidar occupancy grid, A* + reactive planner, a U-shaped dead end, and wheeled/tracked drive types. Run `compare_advanced`. With only the reactive layer the robot drives into the dead end; with A* it replans around it (4/4 points, about 30 cm clearance).

## What it still does not cover
The planar mission (report Sections 1-8) has no terrain height, no torque limits and a reactive planner that can be trapped. The advanced scenario adds height mounds, torque limits, a lidar grid and A*. Even with that, these are NOT covered:
1. Tilt and tip-over: tilt is reported, but there are no roll or pitch dynamics.
2. Getting stuck: only loss of traction and slope/torque limits are modelled, not high-centring or sinking.
3. Climbing over debris with flippers or legs: the robot only goes around obstacles or over gentle mounds.
4. Real SLAM: the lidar map is binary and static and assumes the robot's pose is known.
5. Statistics: one fixed layout, not many random ones.
Note: the dead-end layout was built on purpose to show that a reactive planner can be trapped. It is a demonstration, not a measured failure rate.

## Likely questions and answers

**How would a wheeled robot cross rubble?**
In the advanced scenario the wheeled robot crosses gentle rubble mounds (slope up to 0.20), loses traction on slopes and patches, and avoids the steep pile. For piles it cannot climb it would not. Real disaster-response robots are mostly tracked, or have flippers or legs, because they have to climb.
My two-wheel robot is the simplest case. The control idea (adapting gains to a changing surface) stays the same for a
tracked robot, but the dynamics model would change.

**What if the robot gets trapped?**
The reactive planner can get trapped. To fix this I would add a global planner such as A* or RRT on the map the robot
builds, and use my reactive layer only for local avoidance. This is the standard two-layer design.

**Why do you model rubble only as slip?**
Slip is the part that directly affects the controller. A slip patch changes the gain of the heading loop, and that is
exactly what the fuzzy scheduler is designed to handle. Terrain height is a different problem, mostly for the
mechanical design and the planner.

**How would you make it more realistic?**
Already done in the advanced scenario: height-map terrain, a lidar occupancy grid, A* + reactive planning, motor torque limits, and a tracked parameter set. Still to do:
1. Full SLAM (pose uncertainty, moving obstacles) and tip-over dynamics.
2. A real track-terrain contact model and stuck/high-centring behaviour.
3. Statistics over many random layouts.
4. Real or Webots data instead of the synthetic CSV.

## Closing line
The project is a validated planar core: dynamics, online planning and adaptive control. The next steps are 3D terrain
and a global planner.
