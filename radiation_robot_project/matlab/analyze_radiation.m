% DATA-DRIVEN HOTSPOT ESTIMATION from a radiation log (CSV: t, x, y, dose_rate).
% The file is SYNTHETIC (logged from the simulated robot + sensor noise). Replace it with a real
% survey log of the same format and the same code estimates the hotspots.
p = robot_params();
T = readtable(fullfile('data', 'radiation_log_SYNTHETIC.csv'));
x = T.x; y = T.y; D = T.dose_rate;
hot_idx = find(D > p.THRESH);
% one 'hot episode' = a run of consecutive samples above the threshold (robot inside one hot zone)
breaks  = [0; find(diff(hot_idx) > 1); numel(hot_idx)];
est = zeros(0,3);
for k = 1:numel(breaks)-1
    ii = hot_idx(breaks(k)+1 : breaks(k+1));
    if numel(ii) < 5, continue; end          % ignore sensor-noise blips (a real hot zone lasts many samples)
    w  = D(ii);
    est(end+1,:) = [sum(w.*x(ii))/sum(w), sum(w.*y(ii))/sum(w), max(w)]; %#ok<SAGROW>   % dose-weighted centroid
end
fprintf('\n=== Hotspots estimated from the data (dose-weighted centroid) ===\n');
for i = 1:size(est,1)
    [dmin, j] = min(hypot(p.LEAKS(:,1) - est(i,1), p.LEAKS(:,2) - est(i,2)));
    fprintf('Hotspot %d: (%.2f, %.2f)  peak %.1f  | nearest true source %d, error %.2f m\n', i, est(i,1), est(i,2), est(i,3), j, dmin);
end
figure('Name', 'Radiation survey');
scatter(x, y, 12, D, 'filled'); hold on; colormap(flipud(autumn)); colorbar; axis equal; grid on;
plot(p.LEAKS(:,1), p.LEAKS(:,2), 'kx', 'MarkerSize', 10, 'LineWidth', 2);
plot(est(:,1), est(:,2), 'bo', 'MarkerSize', 14, 'LineWidth', 1.5);
legend('dose rate along path', 'true source (validation only)', 'estimated hotspot');
title('Dose model D = D_0 + sum S/(1+(r/w)^2): survey data and estimated hotspots');
