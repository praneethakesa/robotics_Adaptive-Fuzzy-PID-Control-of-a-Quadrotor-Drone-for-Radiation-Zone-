% COMPARE_PID  Fixed PID vs Adaptive fuzzy-PID on the SAME nonlinear mission.
% Same robot, same obstacles, same slip, same debris strikes, same noise.
% The ONLY difference: simulate_mission(false) keeps Kp,Ki,Kd constant,
%                      simulate_mission(true)  scales them with fuzzy_scales.m.
% Log columns: 1 t, 2 x, 3 y, 4 th, 5 psi_d, 6 e, 7 u, 8 skp, 9 ski, 10 skd,
%              11 dose, 12 wp, 13 F, 14 M, 15 tauR, 16 tauL, 17 v, 18 w
p = robot_params();
F = simulate_mission(false);     % fixed PID
A = simulate_mission(true);      % adaptive fuzzy-PID
names = {'Fixed PID', 'Adaptive fuzzy-PID'};
R = {F, A};
col = {[0.85 0.1 0.1], [0.0 0.55 0.2]};

%% ---------- metrics ----------
rows = {'Mission time [s]', 'Points reached (of 5)', 'Leaks found (of 3)', 'Closest approach [cm]', ...
        'Mean |heading error| [rad]', 'RMS heading error [rad]', 'Peak |heading error| [rad]', ...
        'Peak wheel torque [N m]', 'Mean |yaw-rate command| [rad/s]', 'Path length [m]', ...
        'IAE after debris strikes [rad s]'};
M = zeros(numel(rows), 2);
for i = 1:2
    r = R{i}; L = r.log; e = L(:,6); t = L(:,1);
    % integral of |e| over the 3 s after each debris strike = how well it recovers from hits
    iae = 0;
    for s = 1:size(p.STRIKES,1)
        idx = t >= p.STRIKES(s,1) & t < p.STRIKES(s,1) + 3;
        iae = iae + sum(abs(e(idx))) * p.DT;
    end
    M(:,i) = [r.time; r.reached; size(r.leaks,1); r.min_clear*100; ...
              mean(abs(e)); sqrt(mean(e.^2)); max(abs(e)); ...
              max(max(abs(L(:,15:16)))); mean(abs(L(:,7))); r.path; iae];
end

fprintf('\n=== Fixed PID vs Adaptive fuzzy-PID (same mission, same disturbances) ===\n');
fprintf('%-34s %12s %20s %14s\n', 'Metric', names{1}, names{2}, 'Change');
for k = 1:numel(rows)
    if abs(M(k,1)) > 1e-9, chg = 100*(M(k,2) - M(k,1))/abs(M(k,1)); else, chg = 0; end
    fprintf('%-34s %12.3f %20.3f %12.1f %%\n', rows{k}, M(k,1), M(k,2), chg);
end
fprintf('(negative change in an error row = adaptive is better)\n');

%% ---------- Figure 1: path ----------
figure('Name', 'Compare 1: route', 'Position', [60 60 1000 380]); hold on; axis equal; grid on;
xlim([-1 25]); ylim([-4 4]);
for z = 1:size(p.SLIP_ZONES,1)
    patch([p.SLIP_ZONES(z,1) p.SLIP_ZONES(z,2) p.SLIP_ZONES(z,2) p.SLIP_ZONES(z,1)], [-4 -4 4 4], ...
          [1 .6 0], 'FaceAlpha', .12, 'EdgeColor', 'none');
end
for i = 1:size(p.OBS,1)
    rectangle('Position', [p.OBS(i,1)-p.OBS(i,3) p.OBS(i,2)-p.OBS(i,3) 2*p.OBS(i,3) 2*p.OBS(i,3)], ...
              'Curvature', [1 1], 'FaceColor', [.5 .5 .5], 'EdgeColor', 'none');
end
h1 = plot(F.log(:,2), F.log(:,3), 'Color', col{1}, 'LineWidth', 1.5);
h2 = plot(A.log(:,2), A.log(:,3), '--', 'Color', col{2}, 'LineWidth', 1.5);
plot(p.WP(:,1), p.WP(:,2), 'd', 'Color', [1 .55 0], 'MarkerFaceColor', [1 .55 0]);
plot(p.LEAKS(:,1), p.LEAKS(:,2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
legend([h1 h2], names, 'Location', 'northwest');
title('Route: both controllers (orange = rubble, grey = obstacles, x = leak, diamonds = inspection points)');
xlabel('x [m]'); ylabel('y [m]');

%% ---------- Figure 2: errors, command, gains ----------
figure('Name', 'Compare 2: control signals', 'Position', [80 80 1000 850]);
subplot(4,1,1); hold on; grid on;
plot(F.log(:,1), rad2deg(F.log(:,6)), 'Color', col{1}); plot(A.log(:,1), rad2deg(A.log(:,6)), 'Color', col{2});
for s = 1:size(p.STRIKES,1), xline(p.STRIKES(s,1), ':k'); end
ylabel('heading error [deg]'); legend(names); title('Heading error (dotted lines = debris strikes)');
subplot(4,1,2); hold on; grid on;
plot(F.log(:,1), F.log(:,7), 'Color', col{1}); plot(A.log(:,1), A.log(:,7), 'Color', col{2});
yline(p.W_MAX, '--k'); yline(-p.W_MAX, '--k');
ylabel('yaw-rate command u [rad/s]'); legend(names); title('Controller output (dashed = saturation limit)');
subplot(4,1,3); hold on; grid on;
plot(A.log(:,1), A.log(:,8), 'b'); plot(A.log(:,1), A.log(:,10), 'm'); plot(A.log(:,1), A.log(:,9), 'Color', [.9 .6 0]);
yline(1, '--r');
ylabel('gain scale'); legend('Kp scale', 'Kd scale', 'Ki scale', 'fixed PID (=1)'); title('Adaptive run: fuzzy scale factors');
subplot(4,1,4); hold on; grid on;
plot(F.log(:,1), F.log(:,17), 'Color', col{1}); plot(A.log(:,1), A.log(:,17), 'Color', col{2});
ylabel('forward speed v [m/s]'); xlabel('t [s]'); legend(names); title('Speed drops on rubble (slip)');

%% ---------- Figure 3: zoom on each debris strike ----------
figure('Name', 'Compare 3: recovery after each strike', 'Position', [100 100 1000 600]);
for s = 1:size(p.STRIKES,1)
    subplot(2,2,s); hold on; grid on;
    ts = p.STRIKES(s,1);
    iF = F.log(:,1) >= ts-1 & F.log(:,1) <= ts+5;
    iA = A.log(:,1) >= ts-1 & A.log(:,1) <= ts+5;
    plot(F.log(iF,1), rad2deg(F.log(iF,6)), 'Color', col{1}, 'LineWidth', 1.3);
    plot(A.log(iA,1), rad2deg(A.log(iA,6)), 'Color', col{2}, 'LineWidth', 1.3);
    xline(ts, ':k'); title(sprintf('Strike %d at t = %.0f s (%+.1f rad/s)', s, ts, p.STRIKES(s,2)));
    xlabel('t [s]'); ylabel('error [deg]');
    if s == 1, legend(names, 'Location', 'best'); end
end

%% ---------- Figure 4: bar chart of main metrics ----------
figure('Name', 'Compare 4: summary bars', 'Position', [120 120 900 400]);
sel = [5 6 7 11];
b = bar(M(sel,:)); grid on;
b(1).FaceColor = col{1}; b(2).FaceColor = col{2};
set(gca, 'XTickLabel', {'Mean |e|', 'RMS e', 'Peak |e|', 'IAE after strikes'});
legend(names); ylabel('value [rad or rad s]'); title('Lower is better');
