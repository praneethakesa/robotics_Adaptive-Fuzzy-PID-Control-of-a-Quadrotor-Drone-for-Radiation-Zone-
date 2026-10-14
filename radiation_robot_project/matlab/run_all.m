% Runs everything in order. Put this folder (with the data subfolder) as MATLAB's current folder.
clear; clc; close all;
analyze_stability;      % transfer functions, Routh-Hurwitz, poles, Bode margins
identify_actuator;      % fit tau and slip ratio kappa from a data file
run_mission;            % nonlinear dynamic-route mission, fixed vs adaptive
analyze_radiation;      % hotspot estimation from a radiation log
compare_pid;            % side-by-side metrics table + plots: fixed PID vs adaptive fuzzy-PID
compare_advanced;       % advanced scenario: terrain, torque limits, lidar map, A* + reactive, dead end (takes a few minutes)
