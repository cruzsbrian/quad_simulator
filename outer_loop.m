function [ eta_d, control_thrust ] = outer_loop(pos_d, pos, v)

k_p = 1;
k_v = 1;

eta_d = [0; pi / 6; pi / 6;];

v_d = k_p * (pos_d - pos);

thrust = k_v * (v_d - v);
control_thrust = 3;

end
