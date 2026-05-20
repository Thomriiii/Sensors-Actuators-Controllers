function results = furuta_main()
%FURUTA_MAIN Build, simulate and plot a rotary inverted pendulum model.
%
% The model uses the revised geometry documented in the report and
% estimates mass properties by assuming aluminium components. A local
% balance controller is synthesised from a numerical linearisation of the
% nonlinear plant and then applied to the full nonlinear model so the
% updated rig geometry can be assessed against the same 8 deg
% near-upright disturbance used in the report.

scriptDir = fileparts(mfilename("fullpath"));
projectDir = fileparts(scriptDir);
figDir = fullfile(projectDir, "report", "figures");

if ~exist(figDir, "dir")
    mkdir(figDir);
end

p = buildParameters();
p.reportContext = buildReportContext();

xEq = zeros(4, 1);
uEq = 0.0;
[A, B] = linearisePlant(p, xEq, uEq);

balancePoles = [-4.5, -5.2, -6.2, -7.5];
K = ackermannGain(A, B, balancePoles);
p.K = K;
p.xRef = xEq;
p.balancePoles = balancePoles;

tEnd = 2.5;
tSpan = [0.0, tEnd];
x0 = [deg2rad(0.0);
      deg2rad(8.0);
      0.0;
      0.0];

opts = odeset("RelTol", 1.0e-8, "AbsTol", 1.0e-9);

openSol = ode45(@(t, x) furuta_dynamics(t, x, p, 0.0), tSpan, x0, opts);
closedSol = ode45(@(t, x) furuta_dynamics(t, x, p, controlTorque(x, p)), tSpan, x0, opts);

t = linspace(tSpan(1), tSpan(2), 1400).';
xOpen = deval(openSol, t).';
xClosed = deval(closedSol, t).';
uClosed = arrayfun(@(idx) controlTorque(xClosed(idx, :).', p), 1:numel(t)).';

metrics = computeMetrics(t, xOpen, xClosed, uClosed, p);

createGeometryFigure(p, figDir);
createResponseFigure(t, xOpen, xClosed, uClosed, figDir);

results = struct();
results.parameters = p;
results.A = A;
results.B = B;
results.K = K;
results.balancePoles = balancePoles;
results.time = t;
results.openLoop = xOpen;
results.closedLoop = xClosed;
results.controlTorque = uClosed;
results.metrics = metrics;
results.reportContext = p.reportContext;

save(fullfile(scriptDir, "furuta_results.mat"), "results");
writeSummary(fullfile(scriptDir, "furuta_summary.txt"), p, K, balancePoles, metrics);

fprintf("Generated figures in: %s\n", figDir);
if isnan(metrics.settlingTimeSeconds)
    fprintf("Closed-loop settling time (|alpha| < %.1f deg): not achieved\n", ...
        metrics.alphaBandDeg);
else
    fprintf("Closed-loop settling time (|alpha| < %.1f deg): %.3f s\n", ...
        metrics.alphaBandDeg, metrics.settlingTimeSeconds);
end
fprintf("Peak closed-loop pendulum deviation: %.2f deg\n", metrics.peakAlphaClosedDeg);
fprintf("Peak closed-loop arm excursion: %.2f deg\n", metrics.peakThetaClosedDeg);
fprintf("Peak control torque: %.3f N m\n", metrics.peakTorqueNm);
fprintf("Balance capture achieved: %s\n", tfString(metrics.balanceRecovered));
end

function p = buildParameters()
% Geometric dimensions from the measured coursework rig.
p.Lr = 0.125;           % Arm length [m]
p.armWidth = 0.024;     % Arm width [m]
p.armThickness = 0.004; % Arm thickness [m]

p.rodWidth = 0.008;         % Pendulum rod width [m]
p.rodThickness = 0.005;     % Pendulum rod thickness [m]
p.hubOuterDiameter = 0.018; % Hub OD [m]
p.hubLength = 0.014;        % Hub length [m]
p.hubBore = 0.006;          % Shaft bore [m]
p.totalPendulumHeight = 0.280;
p.rodLength = p.totalPendulumHeight - p.hubLength;

% Practical modelling assumptions.
p.density = 2700.0;   % Aluminium density [kg/m^3]
p.g = 9.81;
p.br = 1.20e-3;       % Arm viscous damping [N m s/rad]
p.bp = 8.00e-4;       % Pendulum viscous damping [N m s/rad]
p.tauMax = 0.18;      % Equivalent actuator saturation [N m]

% Encoder and embedded hardware details retained for documentation.
p.encoderPulsesPerRev = 600;
p.encoderCountsPerRev = 4 * p.encoderPulsesPerRev;
p.hardwareController = "Arduino Uno Rev3 + Motor Shield";
p.hardwareMotor = "42BYGH162-A-21DH";
p.hardwareEncoder = "E38S6G5-600B-G24N";
p.armAngleMeasuredDirectly = false;
p.armAngleEstimateSource = "Estimated in hardware from commanded step count";
p.primaryHardwareIssue = "Stepper skipped steps observed during aggressive motion";

% Derived arm properties.
armVolume = p.Lr * p.armWidth * p.armThickness;
p.mr = p.density * armVolume;
p.JrGeom = (1.0 / 3.0) * p.mr * p.Lr^2;
p.JrAllowance = 2.50e-5; % Extra inertia for coupler and motor-side fixtures.
p.Jr = p.JrGeom + p.JrAllowance;

% Derived pendulum properties.
rodVolume = p.rodLength * p.rodWidth * p.rodThickness;
hubVolume = (pi / 4.0) * (p.hubOuterDiameter^2 - p.hubBore^2) * p.hubLength;

p.mRod = p.density * rodVolume;
p.mHub = p.density * hubVolume;
p.mp = p.mRod + p.mHub;

% The pivot is approximated at the top face of the hub.
zHubCom = 0.5 * p.hubLength;
zRodCom = p.hubLength + 0.5 * p.rodLength;
p.lp = (p.mHub * zHubCom + p.mRod * zRodCom) / p.mp;

% Pendulum inertia about its own centre of mass around the swing axis.
JrodCentroid = (1.0 / 12.0) * p.mRod * (p.rodLength^2 + p.rodWidth^2);
JhubCentroid = (1.0 / 12.0) * p.mHub * (3.0 * (0.5 * p.hubOuterDiameter)^2 + p.hubLength^2);
JrodShift = p.mRod * (zRodCom - p.lp)^2;
JhubShift = p.mHub * (zHubCom - p.lp)^2;
p.Jp = JrodCentroid + JhubCentroid + JrodShift + JhubShift;
end

function context = buildReportContext()
context = struct();
context.projectFocus = "Local MATLAB balance assessment for the revised 125 mm arm rig";
context.fabrication = "3D printed on Bambu printers with fixtures designed around the stepper motor and encoder";
context.hardwareNotes = [
    "Physical rig uses an Arduino Uno Rev3 with Motor Shield Rev3"
    "Actuation is provided by a 42BYGH162-A-21DH stepper motor"
    "Pendulum angle sensing is provided by an E38S6G5-600B-G24N incremental encoder"
    "Arm angle is inferred in hardware from commanded steps rather than measured directly"
    "Skipped motor steps were observed on hardware and are not explicitly modelled in the nonlinear plant"
];
context.reportConclusion = [
    "The revised 125 mm arm geometry changes the leverage and controller demand relative to the previous model"
    "Torque saturation is expected to dominate before the controller can recover the 8 deg disturbance"
];
end

function [A, B] = linearisePlant(p, xEq, uEq)
f = @(x, u) furuta_dynamics(0.0, x, p, u);

nx = numel(xEq);
A = zeros(nx, nx);
B = zeros(nx, 1);

dx = [1e-6; 1e-6; 1e-6; 1e-6];
du = 1e-6;

for k = 1:nx
    e = zeros(nx, 1);
    e(k) = dx(k);
    A(:, k) = (f(xEq + e, uEq) - f(xEq - e, uEq)) / (2.0 * dx(k));
end

B(:, 1) = (f(xEq, uEq + du) - f(xEq, uEq - du)) / (2.0 * du);
end

function K = ackermannGain(A, B, desiredPoles)
n = size(A, 1);
ctrbMatrix = zeros(n, n);

for k = 1:n
    ctrbMatrix(:, k) = A^(k - 1) * B;
end

if rank(ctrbMatrix) < n
    error("The linearised Furuta model is not controllable at the chosen equilibrium.");
end

coeffs = poly(desiredPoles);
phiA = zeros(size(A));

for k = 1:(n + 1)
    powerTerm = n + 1 - k;
    phiA = phiA + coeffs(k) * A^powerTerm;
end

selector = zeros(1, n);
selector(end) = 1.0;
K = real(selector / ctrbMatrix * phiA);
end

function tau = controlTorque(x, p)
tau = -p.K * (x - p.xRef);
tau = max(min(tau, p.tauMax), -p.tauMax);
end

function metrics = computeMetrics(t, xOpen, xClosed, uClosed, p)
alphaClosedDeg = rad2deg(xClosed(:, 2));
thetaClosedDeg = rad2deg(xClosed(:, 1));
alphaOpenDeg = rad2deg(xOpen(:, 2));
thetaOpenDeg = rad2deg(xOpen(:, 1));

alphaBandDeg = 2.0;
idxSettle = find(abs(alphaClosedDeg) > alphaBandDeg, 1, "last");
if isempty(idxSettle)
    settlingTimeSeconds = 0.0;
elseif idxSettle == numel(t)
    settlingTimeSeconds = NaN;
else
    settlingTimeSeconds = t(idxSettle + 1);
end

metrics = struct();
metrics.alphaBandDeg = alphaBandDeg;
metrics.settlingTimeSeconds = settlingTimeSeconds;
metrics.peakAlphaClosedDeg = max(abs(alphaClosedDeg));
metrics.peakThetaClosedDeg = max(abs(thetaClosedDeg));
metrics.peakAlphaOpenDeg = max(abs(alphaOpenDeg));
metrics.peakThetaOpenDeg = max(abs(thetaOpenDeg));
metrics.peakTorqueNm = max(abs(uClosed));
metrics.finalAlphaDeg = alphaClosedDeg(end);
metrics.finalThetaDeg = thetaClosedDeg(end);
metrics.peakThetaClosedTurns = metrics.peakThetaClosedDeg / 360.0;
metrics.finalThetaTurns = metrics.finalThetaDeg / 360.0;
metrics.balanceRecovered = ~isnan(settlingTimeSeconds) && abs(metrics.finalAlphaDeg) <= alphaBandDeg;
metrics.torqueLimited = metrics.peakTorqueNm >= 0.98 * p.tauMax;
metrics.timeAtSaturationFraction = mean(abs(uClosed) >= 0.98 * metrics.peakTorqueNm);
end

function createGeometryFigure(p, figDir)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 760]);
ax = axes(fig);
hold(ax, "on");
axis(ax, "equal");
axis(ax, [-0.04, 0.22, -0.32, 0.05]);
axis(ax, "off");

baseX = 0.0;
baseY = 0.0;
tipX = p.Lr;

plot(ax, [baseX, tipX], [baseY, baseY], "LineWidth", 10, "Color", [0.20, 0.47, 0.72]);
plot(ax, [tipX, tipX], [0.0, -p.hubLength], "LineWidth", 8, "Color", [0.85, 0.33, 0.10]);
plot(ax, [tipX, tipX], [-p.hubLength, -p.hubLength - p.rodLength], "LineWidth", 6, "Color", [0.15, 0.15, 0.15]);

plot(ax, baseX, baseY, "ko", "MarkerFaceColor", "k", "MarkerSize", 8);
text(baseX - 0.006, 0.016, "Motor axis", "FontSize", 12, "FontWeight", "bold");
text(tipX + 0.008, -0.004, "Pendulum pivot", "FontSize", 12, "FontWeight", "bold");

drawDimension(ax, [0.0, -0.028], [p.Lr, -0.028], sprintf("%.0f mm arm length", p.Lr * 1e3));
drawDimension(ax, [p.Lr + 0.024, -p.hubLength], [p.Lr + 0.024, -p.hubLength - p.rodLength], "");
drawDimension(ax, [p.Lr + 0.050, 0.0], [p.Lr + 0.050, -p.totalPendulumHeight], "");

text(0.018, 0.028, sprintf("Arm section: %.0f x %.0f mm", p.armWidth * 1e3, p.armThickness * 1e3), ...
    "FontSize", 11, "Color", [0.20, 0.47, 0.72], "FontWeight", "bold");
text(p.Lr + 0.070, -0.048, sprintf("Hub: OD %.0f mm, bore %.1f mm, length %.0f mm", ...
    p.hubOuterDiameter * 1e3, p.hubBore * 1e3, p.hubLength * 1e3), ...
    "FontSize", 11, "Color", [0.85, 0.33, 0.10], "FontWeight", "bold");
text(p.Lr + 0.070, -0.092, sprintf("Rod section: %.0f x %.0f mm", p.rodWidth * 1e3, p.rodThickness * 1e3), ...
    "FontSize", 11, "Color", [0.15, 0.15, 0.15], "FontWeight", "bold");
text(p.Lr + 0.002, -0.124, sprintf("%.0f mm rod length", p.rodLength * 1e3), "FontSize", 10, "BackgroundColor", "w");
text(p.Lr + 0.066, -0.136, sprintf("%.0f mm overall", p.totalPendulumHeight * 1e3), "FontSize", 10, "BackgroundColor", "w");

exportgraphics(fig, fullfile(figDir, "furuta_geometry.pdf"), "ContentType", "vector");
exportgraphics(fig, fullfile(figDir, "furuta_geometry.png"), "Resolution", 300);
close(fig);
end

function createResponseFigure(t, xOpen, xClosed, uClosed, figDir)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1000, 780]);
tiledlayout(fig, 3, 1, "TileSpacing", "compact", "Padding", "compact");

thetaOpenDeg = rad2deg(xOpen(:, 1));
thetaClosedDeg = rad2deg(xClosed(:, 1));
alphaOpenDeg = rad2deg(xOpen(:, 2));
alphaClosedDeg = rad2deg(xClosed(:, 2));

nexttile;
plot(t, thetaOpenDeg, "--", "LineWidth", 1.8, "Color", [0.85, 0.33, 0.10]);
hold on;
plot(t, thetaClosedDeg, "-", "LineWidth", 1.8, "Color", [0.00, 0.45, 0.74]);
grid on;
ylabel("Arm angle [deg]");
legend("Open loop", "Closed loop", "Location", "best");
title("Updated 125 mm Arm Furuta Model: Open-Loop and Local Balance Response");

nexttile;
plot(t, alphaOpenDeg, "--", "LineWidth", 1.8, "Color", [0.85, 0.33, 0.10]);
hold on;
plot(t, alphaClosedDeg, "-", "LineWidth", 1.8, "Color", [0.00, 0.45, 0.74]);
grid on;
ylabel("Pendulum angle [deg]");
legend("Open loop", "Closed loop", "Location", "best");

nexttile;
plot(t, uClosed, "-", "LineWidth", 1.8, "Color", [0.47, 0.67, 0.19]);
grid on;
xlabel("Time [s]");
ylabel("Torque [N m]");
legend("Closed-loop torque", "Location", "best");

exportgraphics(fig, fullfile(figDir, "furuta_response.pdf"), "ContentType", "vector");
exportgraphics(fig, fullfile(figDir, "furuta_response.png"), "Resolution", 300);
close(fig);
end

function drawDimension(ax, p1, p2, labelText)
plot(ax, [p1(1), p2(1)], [p1(2), p2(2)], "k-", "LineWidth", 1.0);

v = p2 - p1;
vn = v / norm(v);
arrowScale = 0.0045;
quiver(ax, p1(1), p1(2), arrowScale * vn(1), arrowScale * vn(2), 0, ...
    "Color", "k", "LineWidth", 1.0, "MaxHeadSize", 1.8);
quiver(ax, p2(1), p2(2), -arrowScale * vn(1), -arrowScale * vn(2), 0, ...
    "Color", "k", "LineWidth", 1.0, "MaxHeadSize", 1.8);

if ~isempty(labelText)
    mid = 0.5 * (p1 + p2);
    offset = 0.007 * [-vn(2), vn(1)];
    text(mid(1) + offset(1), mid(2) + offset(2), labelText, ...
        "FontSize", 11, "HorizontalAlignment", "center", "BackgroundColor", "w");
end
end

function writeSummary(summaryPath, p, K, desiredPoles, metrics)
fid = fopen(summaryPath, "w");
if fid < 0
    error("Unable to open %s for writing.", summaryPath);
end

cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "Rotary inverted pendulum MATLAB model summary\n");
fprintf(fid, "===========================================\n\n");

fprintf(fid, "Assumptions\n");
fprintf(fid, "- Material density: %.0f kg/m^3\n", p.density);
fprintf(fid, "- Equivalent actuator saturation: %.3f N m\n", p.tauMax);
fprintf(fid, "- Encoder resolution: %d pulses/rev (%d quadrature counts/rev)\n\n", ...
    p.encoderPulsesPerRev, p.encoderCountsPerRev);

fprintf(fid, "Report-aligned hardware context\n");
fprintf(fid, "- Scenario: %s\n", p.reportContext.projectFocus);
fprintf(fid, "- Fabrication note: %s\n", p.reportContext.fabrication);
fprintf(fid, "- Controller hardware: %s\n", p.hardwareController);
fprintf(fid, "- Motor: %s\n", p.hardwareMotor);
fprintf(fid, "- Pendulum encoder: %s\n", p.hardwareEncoder);
fprintf(fid, "- Arm angle feedback: %s\n", p.armAngleEstimateSource);
fprintf(fid, "- Known hardware issue: %s\n\n", p.primaryHardwareIssue);

fprintf(fid, "Derived geometry and inertia\n");
fprintf(fid, "- Arm length Lr: %.3f m\n", p.Lr);
fprintf(fid, "- Arm mass mr: %.5f kg\n", p.mr);
fprintf(fid, "- Arm inertia Jr: %.8f kg m^2\n", p.Jr);
fprintf(fid, "- Pendulum rod length Lrod: %.3f m\n", p.rodLength);
fprintf(fid, "- Overall height Ltot: %.3f m\n", p.totalPendulumHeight);
fprintf(fid, "- Pendulum mass mp: %.5f kg\n", p.mp);
fprintf(fid, "- Pendulum COM offset lp: %.5f m\n", p.lp);
fprintf(fid, "- Pendulum inertia Jp: %.8f kg m^2\n\n", p.Jp);

fprintf(fid, "Controller\n");
fprintf(fid, "- Desired poles: [%.2f %.2f %.2f %.2f]\n", desiredPoles);
fprintf(fid, "- K = [%.6f %.6f %.6f %.6f]\n\n", K(1), K(2), K(3), K(4));

fprintf(fid, "Closed-loop response metrics\n");
if isnan(metrics.settlingTimeSeconds)
    fprintf(fid, "- Settling time within %.1f deg: not achieved\n", metrics.alphaBandDeg);
else
    fprintf(fid, "- Settling time within %.1f deg: %.4f s\n", metrics.alphaBandDeg, metrics.settlingTimeSeconds);
end
fprintf(fid, "- Peak pendulum deviation: %.4f deg\n", metrics.peakAlphaClosedDeg);
fprintf(fid, "- Peak arm excursion: %.4f deg\n", metrics.peakThetaClosedDeg);
fprintf(fid, "- Peak control torque: %.4f N m\n", metrics.peakTorqueNm);
fprintf(fid, "- Final pendulum angle: %.4f deg\n", metrics.finalAlphaDeg);
fprintf(fid, "- Final arm angle: %.4f deg\n", metrics.finalThetaDeg);
fprintf(fid, "- Final arm rotation: %.4f turns\n", metrics.finalThetaTurns);
fprintf(fid, "- Time at torque saturation (98%% of limit or above): %.2f %%\n", ...
    100.0 * metrics.timeAtSaturationFraction);
fprintf(fid, "- Balance recovery achieved: %s\n", tfString(metrics.balanceRecovered));
fprintf(fid, "- Torque-limited response observed: %s\n\n", tfString(metrics.torqueLimited));

fprintf(fid, "Interpretation\n");
if metrics.balanceRecovered
    fprintf(fid, "- With the 125 mm arm, the local balance controller recovers the 8 deg disturbance in simulation and remains close to the upright equilibrium.\n");
else
    fprintf(fid, "- With the 125 mm arm, the local balance controller does not recover the 8 deg disturbance in simulation, so the gain set or capture condition still needs revision.\n");
end
if metrics.torqueLimited
    fprintf(fid, "- The simulated actuator reaches its saturation limit, so the response is still leverage-limited even before hardware losses are considered.\n");
else
    fprintf(fid, "- The simulated actuator remains well below the saturation limit, indicating that the longer arm improves control authority in the ideal model.\n");
end
fprintf(fid, "- Hardware performance is expected to be worse than the ideal simulation because skipped steps and arm-angle estimation drift are not explicitly modelled here.\n");
fprintf(fid, "- Swing-up behaviour is handled in the Arduino implementation rather than in this local MATLAB balance script.\n");
end

function textValue = tfString(flag)
if flag
    textValue = "yes";
else
    textValue = "no";
end
end
