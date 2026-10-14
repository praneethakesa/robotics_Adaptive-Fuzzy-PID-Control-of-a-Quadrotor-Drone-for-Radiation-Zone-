% DEMO: key dynamics and fuzzy-scheduler values in one run
p = robot_params();

fprintf('\n=== Fuzzy gain scheduler: (|e|, |de|) -> scales ===\n');
cases = [0.05 0.1; 0.30 0.7; 0.50 1.0];
for i = 1:size(cases,1)
    [kp, ki, kd] = fuzzy_scales(cases(i,1), cases(i,2));
    fprintf('e=%.2f de=%.2f  ->  Kp x%.2f  Ki x%.2f  Kd x%.2f\n', cases(i,:), kp, ki, kd);
end

fprintf('\n=== Newton-Euler dynamics at start-up (v=0, w=0, v_ref=%.1f, w_ref=1.0) ===\n', p.V_CMD);
for kappa = [1.0 0.6]
    [vdot, wdot, F, M] = robot_accel(0, 0, p.V_CMD, 1.0, kappa, p);
    tauR = p.r*F/2 + p.r*M/p.L;
    tauL = p.r*F/2 - p.r*M/p.L;
    fprintf('kappa=%.1f: vdot=%.2f m/s^2  wdot=%.2f rad/s^2  F=%.1f N  M=%.2f Nm  tauR=%.2f  tauL=%.2f Nm\n', ...
            kappa, vdot, wdot, F, M, tauR, tauL);
end
fprintf('Rubble (kappa=0.6) removes 40%% of the actuation: same command, less force and slower acceleration.\n');
