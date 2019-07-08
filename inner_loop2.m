function [control_1, control_2, control_3 ]  = inner_loop2(phi_d, theta_d, psi_d, phi, theta, psi, p, q, r)

Ixx = 1.0181101*10^(-2);   % Moments (kg*m^2)
Iyy = 0.97079192*10^(-2);
Izz = 1.8834745*10^(-2);

% gain on omega_d, not in paper
% relates error in attitude to desired angular velocity
k = [5.0, 0, 0; 
    0, 5.0, 0; 
    0, 0, 2.8];

lambda = [20.00, 0, 0; 
        0, 20.00, 0; 
        0, 0, 20.00];

% K = [0, 0, 0; 
%     0, 0, 0; 
%     0, 0, 0];

K = [750, 0, 0; 
    0, 750, 0; 
    0, 0, 750];

eta_d = [phi_d; theta_d; psi_d];
eta = [phi; theta; psi];
omega = [p; q; r];

% transformation from body frame to intertial frame and inverse (Eq 3.19)
Z = [1, sin(eta(1))*tan(eta(2)), cos(eta(1))*tan(eta(2));
    0,  cos(eta(1)),            -sin(eta(1));
    0,  sin(eta(1))/cos(eta(2)), cos(eta(1))/cos(eta(2))];

invZ = [1,  0,               - sin(eta(2));
    0,  cos(eta(1)),     sin(eta(1))*cos(eta(2));
    0,  - sin(eta(1)),   cos(eta(1))*cos(eta(2))];


omega_d = k*(eta_d - eta);
omega_d = invZ*omega_d;
eta_dot = Z*omega;

% time derivative of invZ
invZ_dot = [0, 0,                        -cos(eta(2))*eta_dot(2);
    0, - sin(eta(1))*eta_dot(1),  cos(eta(1))*cos(eta(2))*eta_dot(1) - sin(eta(1))*sin(eta(2))*eta_dot(2);
    0, - cos(eta(1))*eta_dot(1), - sin(eta(1))*cos(eta(2))*eta_dot(1) - cos(eta(1))*sin(eta(2))*eta_dot(2)];

% set desired angular acceleration as 0
omega_d_dot = [0; 0; 0];

omega_r = omega_d + invZ * lambda*(eta_d - eta);
omega_r_dot = omega_d_dot + lambda*(omega_d - omega) + invZ_dot*lambda*(eta_d - eta);

control_1 = Ixx * omega_r_dot(1) - (Iyy*omega(2)*omega_r(3) - Izz*omega(3)*omega_r(2)) + K(1,1)*(omega_r(1) - omega(1));
control_2 = Iyy * omega_r_dot(2) - (Izz*omega(3)*omega_r(1) - Ixx*omega(1)*omega_r(3)) + K(2,2)*(omega_r(2) - omega(2));
control_3 = Izz * omega_r_dot(3) - (Ixx*omega(1)*omega_r(2) - Iyy*omega(2)*omega_r(1)) + K(3,3)*(omega_r(3) - omega(3));

