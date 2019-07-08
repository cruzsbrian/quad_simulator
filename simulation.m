function main
    eta_0 = [0; 0; 0];
    omega_0 = [0; 0; 0];
    x0 = [eta_0; omega_0];

    eta_d = [0; -pi / 6; pi / 6];

    thrust = 3;

    options = odeset('MaxStep', 0.1);
    [t, x] = ode45(@(t, x) F(t, x, eta_d, thrust), [0 1], x0, options);

    plot(t, x(:, 1:3));
    legend('phi', 'theta', 'psi');
end

function x_dot = F(~, x, eta_d, thrust)
    eta = x(1:3);
    omega = x(4:6);

    control_torque = inner_loop(eta_d, eta, omega);

    pwm = mixer(thrust, control_torque);

    [eta_dot, omega_dot] = dynamics(pwm, eta, omega);

    x_dot = [eta_dot; omega_dot];
end
