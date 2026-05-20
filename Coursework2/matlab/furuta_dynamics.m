function xdot = furuta_dynamics(~, x, p, tau)
%FURUTA_DYNAMICS Nonlinear dynamics of a rotary inverted pendulum.
%   State vector x = [theta; alpha; theta_dot; alpha_dot], where alpha = 0
%   corresponds to the upright pendulum position. The input tau is treated
%   as an idealised equivalent arm torque; skipped-step behaviour from the
%   physical stepper drive is intentionally excluded from this plant model.

theta = x(1); %#ok<NASGU>
alpha = x(2);
thetaDot = x(3);
alphaDot = x(4);

s = sin(alpha);
c = cos(alpha);

M11 = p.Jr + p.mp * (p.Lr^2 + p.lp^2 * s^2);
M12 = p.mp * p.Lr * p.lp * c;
M22 = p.Jp + p.mp * p.lp^2;

M = [M11, M12;
     M12, M22];

h1 = 2.0 * p.mp * p.lp^2 * s * c * thetaDot * alphaDot ...
   - p.mp * p.Lr * p.lp * s * alphaDot^2 ...
   + p.br * thetaDot;

h2 = -p.mp * p.lp^2 * s * c * thetaDot^2 ...
   - p.mp * p.g * p.lp * s ...
   + p.bp * alphaDot;

qdd = M \ ([tau; 0.0] - [h1; h2]);

xdot = [thetaDot;
        alphaDot;
        qdd(1);
        qdd(2)];
end
