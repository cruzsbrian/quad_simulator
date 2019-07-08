function test
    eta_0 = [0; 0; 0];
    omega_0 = [0; 0; 0];
    x0 = [eta_0; omega_0];

    eta_d = [0; pi / 6; pi / 6];

    thrust = 3;

    c = inner_loop(eta_d, eta_0, omega_0);
    [c1, c2, c3] = inner_loop2(eta_d(1), eta_d(2), eta_d(3), eta_0(1), eta_0(2), eta_0(3), omega_0(1), omega_0(2), omega_0(3));

    pwm = mixer(thrust, c);
    [p1, p2, p3, p4] = mixer2(thrust, c1, c2, c3);

    [eta_dot, omega_dot] = dynamics(pwm, eta_0, omega_0);
    [phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot] = dynamics2(p1, p2, p3, p4, eta_0(1), eta_0(2), eta_0(3), omega_0(1), omega_0(2), omega_0(3));

    disp('New omega_dot');
    disp(omega_dot);
    disp('Correct omega_dot');
    disp([p_dot; q_dot; r_dot]);
end
