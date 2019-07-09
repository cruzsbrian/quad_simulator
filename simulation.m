function main
    pos_0 = [0; 0; 0];
    v_0 = [0; 0; 0];
    eta_0 = [0; 0; 0];
    omega_0 = [0; 0; 0];
    x0 = [pos_0; v_0; eta_0; omega_0];

    eta_d = [0; 0; pi / 4];

    thrust = 3;

    options = odeset('MaxStep', 0.1);
    [t, x] = ode45(@(t, x) F(t, x, eta_d, thrust), [0 1], x0, options);

    subplot(2,1,1);
    plot(t, x(:, 1:3));
    legend('pos_x', 'pos_y', 'pos_z');
    title('Position');

    subplot(2,1,2);
    plot(t, x(:, 7:9));
    legend('phi', 'theta', 'psi');
    title('Attitude');
end

function x_dot = F(t, x, eta_d, thrust)
    pos = x(1:3);
    v = x(4:6);
    eta = x(7:9);
    omega = x(10:12);

    control_torque = inner_loop(eta_d, eta, omega);

    pwm = mixer(thrust, control_torque);

    [pos_dot, v_dot, eta_dot, omega_dot] = dynamics(pwm, pos, v, eta, omega);

    x_dot = [pos_dot; v_dot; eta_dot; omega_dot];
end
