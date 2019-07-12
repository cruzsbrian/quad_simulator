function [ eta_d, control_thrust ] = outer_loop(psi_d, pos_d, pos, v)

mass = 0.028;
g = [0; 0; -9.81];

K_pos = 0.7;
kp = 0.1;

v_d = K_pos * (pos_d - pos);

thrust = kp * (v_d - v) - mass * g;
control_thrust = norm(thrust);

phi_d = asin((thrust(1) * sin(psi_d) - thrust(2) * cos(psi_d)) / norm(thrust));
theta_d = atan((thrust(1) * cos(psi_d) + thrust(2) * sin(psi_d)) / thrust(3));
eta_d = [phi_d; theta_d; psi_d];

end
