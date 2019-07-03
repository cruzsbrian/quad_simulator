function [ control_torque ] = inner_loop(eta_d, eta, omega)

% Physical constants
Ixx = 1.0181101*10^(-2);
Iyy = 0.97079192*10^(-2);
Izz = 1.8834745*10^(-2);
J = [ Ixx, 0, 0;
      0, Iyy, 0;
      0, 0, Izz ];

% gain on omega_d
% relates error in attitude to desired angular velocity
k = [ 5, 0, 0;
      0, 5, 0;
      0, 0, 2.8 ];

lambda = [ 20, 0, 0;
           0, 20, 0
           0, 0, 20 ];

K = [ 750, 0, 0;
      0, 750, 0;
      0, 0, 750 ];

% Transformation from body frame to intertial frame, inverse
Z = [ 1, sin(eta(1))*tan(eta(2)), cos(eta(1))*tan(eta(2));
      0, cos(eta(1)),             -sin(eta(1));
      0, sin(eta(1))/cos(eta(2)), cos(eta(1))/cos(eta(2)) ];

invZ = [ 1,  0,             -sin(eta(2));
         0,  cos(eta(1)),   sin(eta(1))*cos(eta(2));
         0,  -sin(eta(1)), cos(eta(1))*cos(eta(2))];

% Get desired angular speed (proportional to error in attitude)
omega_d = invZ * k * (eta_d - eta);

% Get changes in Euler angles from angular speed using transformation
eta_dot = Z * omega;

% time derivative of invZ
invZ_dot = [ 0, 0,                        -cos(eta(2))*eta_dot(2);
             0, - sin(eta(1))*eta_dot(1),  cos(eta(1))*cos(eta(2))*eta_dot(1) - sin(eta(1))*sin(eta(2))*eta_dot(2);
             0, - cos(eta(1))*eta_dot(1), -sin(eta(1))*cos(eta(2))*eta_dot(1) - cos(eta(1))*sin(eta(2))*eta_dot(2) ];

% set desired angular acceleration as 0
omega_d_dot = [0; 0; 0];

omega_r = omega_d + invZ * lambda * (eta_d - eta);
omega_r_dot = omega_d_dot + lambda*(omega_d - omega) + invZ_dot * lambda * (eta_d - eta);

control_torque = J * omega_r_dot - cross(J * omega, omega_r) + K * (omega_r - omega);
