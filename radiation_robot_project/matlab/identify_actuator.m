% SYSTEM IDENTIFICATION from a data file (CSV): fit the first-order yaw-rate servo + slip ratio.
%   Model:  y(t) = kappa * u0 * (1 - exp(-(t - t0)/tau)),   t >= t0
%   y = measured yaw rate, u0 = commanded step, kappa = slip/traction ratio, tau = servo time constant.
% The file below is SYNTHETIC (generated from the model + noise) so the pipeline runs out of the box.
% To connect REAL data: log (t, yaw-rate command, measured yaw rate, surface) from your robot or
% simulator, save it in the same 4-column format, and change the file name.
p = robot_params();
T = readtable(fullfile('data', 'actuator_step_log_SYNTHETIC.csv'));
surfs = unique(string(T.surface));
fprintf('\n=== Actuator identification from data ===\n');
figure('Name', 'Identification fit');
for i = 1:numel(surfs)
    idx = string(T.surface) == surfs(i);
    t = T.t(idx); u = T.u_cmd(idx); y = T.omega_meas(idx);
    t0 = t(find(u > 0, 1, 'first')); u0 = max(u);
    model = @(th) th(1)*u0*(1 - exp(-max(t - t0, 0)/th(2)));
    cost  = @(th) sum((y - model(th)).^2);
    th    = fminsearch(cost, [0.8 0.3]);
    yfit  = model(th);
    rmse  = sqrt(mean((y - yfit).^2));
    r2    = 1 - sum((y - yfit).^2)/sum((y - mean(y)).^2);
    fprintf('%-8s kappa = %.3f   tau_w = %.3f s   RMSE = %.4f rad/s   R^2 = %.4f\n', surfs(i), th(1), th(2), rmse, r2);
    subplot(numel(surfs), 1, i); hold on; grid on;
    plot(t, y, '.', 'Color', [.6 .6 .6]); plot(t, yfit, 'r', 'LineWidth', 1.5);
    title(sprintf('%s: kappa = %.2f, tau = %.3f s', surfs(i), th(1), th(2))); ylabel('yaw rate [rad/s]');
end
xlabel('t [s]'); legend('measured', 'fitted model');
fprintf('Model in robot_params.m uses tau_w = %.2f s and kappa = 1 (clear) / %.1f (rubble).\n', p.tau_w, p.KAPPA);
fprintf('Torque-level meaning:  K_w = I / tau_w = %.2f N m s/rad\n', p.I/p.tau_w);
