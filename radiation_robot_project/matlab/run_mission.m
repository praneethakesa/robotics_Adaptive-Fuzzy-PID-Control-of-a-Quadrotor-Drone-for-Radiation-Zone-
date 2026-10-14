% Dynamic-route inspection mission: fixed PID vs adaptive fuzzy-PID (nonlinear simulation)
F = simulate_mission(false);
A = simulate_mission(true);
names = {'Fixed PID', 'Adaptive fuzzy-PID'}; R = {F, A};
for i = 1:2
    r = R{i};
    fprintf('%-20s time=%.1fs reached=%d/5 leaks=%d/3 mapped=%d/10 min_clear=%.0fcm mean|e|=%.3frad max|e|=%.2frad\n', ...
        names{i}, r.time, r.reached, size(r.leaks,1), r.known, r.min_clear*100, r.mean_abs_e, r.max_abs_e);
end
P = A.p;
figure('Name', 'Mission', 'Position', [80 60 1000 850]);
subplot(3,1,1); hold on; axis equal; grid on; xlim([-1 25]); ylim([-4 4]);
for z = 1:size(P.SLIP_ZONES,1)
    patch([P.SLIP_ZONES(z,1) P.SLIP_ZONES(z,2) P.SLIP_ZONES(z,2) P.SLIP_ZONES(z,1)], [-4 -4 4 4], [1 .6 0], 'FaceAlpha', .12, 'EdgeColor', 'none');
end
for i = 1:size(P.OBS,1)
    rectangle('Position', [P.OBS(i,1)-P.OBS(i,3) P.OBS(i,2)-P.OBS(i,3) 2*P.OBS(i,3) 2*P.OBS(i,3)], 'Curvature', [1 1], 'FaceColor', [.45 .45 .45], 'EdgeColor', 'none');
end
plot(F.log(:,2), F.log(:,3), 'r', 'LineWidth', 1.2);
scatter(A.log(1:5:end,2), A.log(1:5:end,3), 6, A.log(1:5:end,11), 'filled');
colormap(flipud(autumn)); cb = colorbar; cb.Label.String = 'dose rate';
plot(P.WP(:,1), P.WP(:,2), 'd', 'Color', [1 .55 0], 'MarkerFaceColor', [1 .55 0]);
for i = 1:size(P.WP,1), text(P.WP(i,1), P.WP(i,2)+0.4, sprintf('INSPECT %d', i), 'HorizontalAlignment', 'center', 'FontSize', 8); end
plot(P.LEAKS(:,1), P.LEAKS(:,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot(A.leaks(:,1), A.leaks(:,2), 'ro', 'MarkerSize', 14, 'LineWidth', 1.5);
title('Dynamic route (grey = rubble, x = true leak, circle = leak found by robot, red line = fixed PID)');
subplot(3,1,2); hold on; grid on;
plot(F.log(:,1), rad2deg(F.log(:,6)), 'r'); plot(A.log(:,1), rad2deg(A.log(:,6)), 'g');
ylabel('heading error [deg]'); legend('Fixed PID', 'Adaptive fuzzy-PID');
subplot(3,1,3); hold on; grid on;
plot(A.log(:,1), A.log(:,8)); plot(A.log(:,1), A.log(:,10));
ylabel('fuzzy scale'); xlabel('t [s]'); legend('Kp scale', 'Kd scale');

figure('Name', 'Dynamics: velocities, forces, wheel torques', 'Position', [120 80 900 700]);
subplot(3,1,1); hold on; grid on;
plot(A.log(:,1), A.log(:,17), 'b'); plot(A.log(:,1), A.log(:,18), 'm');
ylabel('v [m/s],  \omega [rad/s]'); legend('forward speed v', 'yaw rate \omega'); title('Adaptive fuzzy-PID run: states from m dv/dt = F - bv,  I d\omega/dt = M - c\omega');
subplot(3,1,2); hold on; grid on;
plot(A.log(:,1), A.log(:,13), 'b'); plot(A.log(:,1), A.log(:,14), 'm');
ylabel('F [N],  M [N m]'); legend('traction force F', 'yaw moment M');
subplot(3,1,3); hold on; grid on;
plot(A.log(:,1), A.log(:,15), 'r'); plot(A.log(:,1), A.log(:,16), 'k');
ylabel('wheel torque [N m]'); xlabel('t [s]'); legend('right wheel', 'left wheel');
