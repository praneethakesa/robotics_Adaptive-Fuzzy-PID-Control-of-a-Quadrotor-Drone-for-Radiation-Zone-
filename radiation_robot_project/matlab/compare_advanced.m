% ADVANCED SCENARIO: dense obstacles + 3D terrain (height mounds) + motor torque limits + lidar map + U-shaped dead end.
% Compares   reactive-only  vs  A* + reactive   x   fixed PID  vs  adaptive fuzzy-PID   x   wheeled vs tracked.
W = world_hard(); a = adv_params(); p = robot_params();
drives = {'wheeled', 'tracked'}; planners = [false true]; ctrls = [false true];
R = cell(2,2,2);
fprintf('\n=== Advanced scenario: %d obstacles, height mounds, torque limits, dead end ===\n', size(W.obstacles,1));
fprintf('%-8s %-14s %-9s %-10s %6s %5s %6s %6s %6s %6s %7s %8s\n', 'drive', 'planner', 'control', 'outcome', 'time', 'pts', 'leaks', 'clear', 'tilt', 'sat%', 'peakT', 'mean|e|');
for d = 1:2
  for g = 1:2
    for c = 1:2
        r = simulate_advanced(W, ctrls(c), planners(g), drives{d});
        R{d,g,c} = r;
        if r.success, outc = 'OK'; elseif r.collided, outc = 'COLLISION'; else, outc = 'TIMEOUT'; end
        if planners(g), pn = 'A*+reactive'; else, pn = 'reactive-only'; end
        if ctrls(c), cn = 'adaptive'; else, cn = 'fixed'; end
        fprintf('%-8s %-14s %-9s %-10s %6.1f %3d/4 %3d/3 %4.0fcm %5.1f %6.1f %7.2f %8.3f\n', drives{d}, pn, cn, outc, ...
                r.time, r.reached, size(r.leaks,1), r.min_clear*100, r.max_tilt, r.sat_pct, r.peak_torque, r.mean_abs_e);
    end
  end
end
fprintf('clear = closest approach to any obstacle surface (negative = hit); tilt = peak terrain tilt [deg];\n');
fprintf('sat%% = share of time a wheel torque hit the motor limit; peakT = peak wheel torque [N m].\n');

% ---- figure 1: routes over the terrain ----
sel = {{1,1,2,'Wheeled, reactive-only (adaptive PID): runs into the dead end'}, ...
       {1,2,2,'Wheeled, A* + reactive: detects the dead end, plans around it, avoids the steep pile'}, ...
       {2,2,2,'Tracked, A* + reactive: higher slope limit and less slip on rubble, shorter faster run'}};
figure('Name', 'Advanced: routes', 'Position', [60 40 1000 900]);
for s = 1:3
    q = sel{s}; r = R{q{1}, q{2}, q{3}};
    subplot(3,1,s); hold on; axis equal; grid on; xlim([-1 27]); ylim([-5 5]);
    contourf(a.CXX, a.CYY, r.slope, [0.2 0.45 2], 'LineStyle', 'none'); colormap(flipud(autumn));
    for i = 1:size(W.obstacles,1)
        o = W.obstacles(i,:);
        rectangle('Position', [o(1)-o(3) o(2)-o(3) 2*o(3) 2*o(3)], 'Curvature', [1 1], 'FaceColor', [.4 .4 .4], 'EdgeColor', 'none');
    end
    plot(r.log(:,2), r.log(:,3), 'g', 'LineWidth', 1.4);
    plot(W.waypoints(:,1), W.waypoints(:,2), 'd', 'Color', [1 .55 0], 'MarkerFaceColor', [1 .55 0]);
    plot(W.leaks(:,1), W.leaks(:,2), 'rx', 'MarkerSize', 9, 'LineWidth', 2);
    title(q{4});
end
% ---- figure 2: fixed vs adaptive on the successful wheeled run ----
Fx = R{1,2,1}; Ad = R{1,2,2};
figure('Name', 'Advanced: fixed vs adaptive', 'Position', [100 80 1000 700]);
subplot(3,1,1); hold on; grid on;
plot(Fx.log(:,1), rad2deg(Fx.log(:,6)), 'r'); plot(Ad.log(:,1), rad2deg(Ad.log(:,6)), 'g');
ylabel('heading error [deg]'); legend('Fixed PID', 'Adaptive fuzzy-PID'); title('A* + reactive, wheeled robot');
subplot(3,1,2); hold on; grid on;
plot(Fx.log(:,1), Fx.log(:,15), 'r'); plot(Ad.log(:,1), Ad.log(:,15), 'g');
tm = a.wheeled.tau_max; plot([0 Ad.time], [tm tm], '--k'); plot([0 Ad.time], [-tm -tm], '--k');
ylabel('right wheel torque [N m]'); legend('Fixed PID', 'Adaptive fuzzy-PID', 'motor limit');
subplot(3,1,3); hold on; grid on;
plot(Ad.log(:,1), Ad.log(:,19), 'b'); plot(Ad.log(:,1), Ad.log(:,20)/100, 'm');
ylabel('z [m], tilt/100 [deg]'); xlabel('t [s]'); legend('terrain height under robot', 'tilt (deg/100)');
