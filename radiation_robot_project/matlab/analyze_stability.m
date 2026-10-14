% LINEARISED HEADING LOOP: transfer functions, Routh-Hurwitz, poles, Bode margins, step response.
% Needs the Control System Toolbox (tf, feedback, margin, step, rlocus).
%
% Heading plant (thdot = w, wdot = (kappa*u - w)/tau_w):   G(s) = kappa / ( s (tau_w s + 1) )
% PID with derivative filter:  C(s) = Kp + Ki/s + Kd s/(Tf s + 1)
% Loop L = C G;  ideal-PID characteristic polynomial:
%   tau_w s^3 + (1 + kappa Kd) s^2 + kappa Kp s + kappa Ki = 0
% Routh-Hurwitz:  all coefficients > 0  and  (1 + kappa Kd) Kp > tau_w Ki
p = robot_params();
s = tf('s');
G0 = 1/(s*(p.tau_w*s + 1));

% name, kappa, Kp-scale, Ki-scale, Kd-scale
cases = { 'Fixed PID, clear floor',                         1.0, 1.0, 1.0, 1.0;
          'Fixed PID, rubble (kappa = 0.6)',                0.6, 1.0, 1.0, 1.0;
          'Adaptive, rubble, large fast error',             0.6, 1.8, 0.0, 1.5;
          'Adaptive, clear floor, small error',             1.0, 0.8, 1.0, 0.8 };

fprintf('\n=== Linearised heading loop: poles and margins ===\n');
Ls = cell(1, size(cases,1)); Ts = Ls;
for i = 1:size(cases,1)
    kappa = cases{i,2}; skp = cases{i,3}; ski = cases{i,4}; skd = cases{i,5};
    Kp = p.KP0*skp; Ki = p.KI0*ski; Kd = p.KD0*skd;
    C  = Kp + Ki/s + Kd*s/(p.Tf*s + 1);
    L  = minreal(kappa*C*G0, 1e-6);
    T  = minreal(feedback(L, 1), 1e-6);
    [Gm, Pm, ~, Wcp] = margin(L);
    a3 = p.tau_w; a2 = 1 + kappa*Kd; a1 = kappa*Kp; a0 = kappa*Ki;
    routh = (a2*a1 - a3*a0) > 0;
    fprintf('%-38s Kp=%.2f Ki=%.3f Kd=%.3f | crossover=%.2f rad/s  PM=%.1f deg  Routh ok=%d\n', ...
            cases{i,1}, Kp, Ki, Kd, Wcp, Pm, routh);
    fprintf('    closed-loop poles: %s\n', mat2str(round(pole(T).', 3)));
    Ls{i} = L; Ts{i} = T;
end

% --- check every corner of the scheduling range (kappa x Kp x Ki x Kd scales) ---
fprintf('\n=== Stability at all corners of the fuzzy scheduling range ===\n');
all_ok = true; worst = inf;
for kappa = [0.6 1.0]
  for skp = [0.8 2.0]
    for ski = [0 1]
      for skd = [0.8 2.0]
        Kp = p.KP0*skp; Ki = p.KI0*ski; Kd = p.KD0*skd;
        den  = conv(conv([1 0 0], [p.tau_w 1]), [p.Tf 1]);
        numr = padd(conv([Kp Ki], [p.Tf 1]), [Kd 0 0]);
        r = roots(padd(den, kappa*numr));
        if ski == 0, [~, kz] = min(abs(r)); r(kz) = []; end   % drop the pole/zero-cancelled s = 0
        worst = min(worst, (1 + kappa*Kd)*kappa*Kp - p.tau_w*kappa*Ki);
        all_ok = all_ok && all(real(r) < 0);
      end
    end
  end
end
fprintf('All 16 corners stable: %d   (smallest Routh margin a2*a1 - a3*a0 = %.3f)\n', all_ok, worst);
fprintf('NOTE: this is a frozen-gain (quasi-static) check. It does not by itself prove stability\n');
fprintf('      of the time-varying fuzzy law; that needs slowly-varying gains or a Lyapunov argument.\n');

figure('Name', 'Bode of the loop'); hold on;
for i = [1 2 3], bodeplot(Ls{i}); end
legend(cases{[1 2 3],1}); grid on;
figure('Name', 'Heading step response (closed loop)'); hold on;
for i = [1 2 3], step(Ts{i}, 12); end
legend(cases{[1 2 3],1}); grid on; title('Closed-loop response to a 1 rad heading step');
figure('Name', 'Root locus vs loop gain (fixed PID)');
rlocus(Ls{1}); grid on; title('Root locus: loop gain scaling of the fixed-PID loop (kappa=1 is the marked design point)');

function c = padd(a, b)
n = max(numel(a), numel(b));
c = [zeros(1, n - numel(a)) a] + [zeros(1, n - numel(b)) b];
end
