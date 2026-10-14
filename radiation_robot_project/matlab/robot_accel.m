function [vdot, wdot, F, M] = robot_accel(v, w, v_ref, w_ref, kappa, p)
% Newton-Euler dynamics with the inner servos:
%   m  vdot = F - b v        F = b v + Kv (kappa v_ref - v)
%   I  wdot = M - c w        M = c w + Kw (kappa w_ref - w)
% kappa = slip/traction ratio (ground speed = kappa * commanded wheel surface speed).
F = p.b*v + p.Kv*(kappa*v_ref - v);      % net traction force  [N]
M = p.c*w + p.Kw*(kappa*w_ref - w);      % net yaw moment      [N m]
vdot = (F - p.b*v) / p.m;                % = (kappa v_ref - v)/tau_v
wdot = (M - p.c*w) / p.I;                % = (kappa w_ref - w)/tau_w
end
