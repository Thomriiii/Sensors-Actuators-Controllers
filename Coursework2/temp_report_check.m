load('matlab/furuta_results.mat');
t = results.time;
a = rad2deg(results.openLoop(:,2));
idx = find(t >= 1.0, 1, 'first');
fprintf('openAlphaAt1s=%.4f\n', a(idx));
fprintf('openFinalAlpha=%.4f\n', a(end));
fprintf('openPeakAbsAlpha=%.4f\n', max(abs(a)));
