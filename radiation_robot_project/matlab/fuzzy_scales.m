function [skp, ski, skd] = fuzzy_scales(e, de)
% Sugeno fuzzy gain scheduler: inputs |e| and |de| -> scale factors for Kp, Ki, Kd
KP_TAB = [0.8 0.9 1.0; 1.2 1.3 1.4; 1.6 1.8 2.0];
KD_TAB = [0.8 1.2 1.6; 0.8 1.4 1.8; 0.8 1.5 2.0];
KI_TAB = [1.0 0.5 0.0];
me = memberships(abs(e)  / 0.6);     % rows of the rule table
md = memberships(abs(de) / 1.5);     % columns of the rule table
W   = me' * md;                      % rule firing strengths (3x3)
skp = sum(sum(W .* KP_TAB));
skd = sum(sum(W .* KD_TAB));
ski = sum(me .* KI_TAB);
end

function m = memberships(u)
u = min(max(u, 0), 1);
m = [max(0, 1 - 2*u), max(0, 1 - 2*abs(u - 0.5)), max(0, 2*u - 1)];   % Small, Medium, Large
end
