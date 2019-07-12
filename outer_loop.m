function [ eta_d, control_thrust ] = outer_loop(pos_d, pos, v)

mass = 0.028;
g = -9.81;

k_p = 2;
k_v = 10;

eta_d = [0; 0; pi / 6];

v_d = k_p * (pos_d - pos);

thrust = k_v * (v_d - v);
control_thrust = thrust(3) - mass * g;

end
