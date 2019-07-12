function [pos_dot, v_dot, eta_dot, omega_dot]= dynamics(rpm, pos, v, eta, omega)

% Physical constants
air_density = 1.225;

mass = 0.028;
g = [0; 0; -9.81];

Ixx = 16.571710*10^(-6);
Iyy = 16.655602*10^(-6);
Izz = 29.261652*10^(-6);
J = [ Ixx, 0, 0;
      0, Iyy, 0;
      0, 0, Izz ];

prop_diameter = 0.045;
arm_length = 0.066;

% Limit rpm to achievable values
rpm = min(max(rpm, 7530), 26993);

rpm_square = rpm.^2;

% Estimate power and thrust coefficients from rpm
CT = 2.79854e-5 + 0.324592 * (1 ./ rpm) - 1.43398e3 * (1 ./ rpm.^2);
CP = 2.33065e-5 + 0.270271 * (1 ./ rpm) - 759.67 * (1 ./ rpm.^2);

% Aerodynamic coefficient matrix from above values
B = [
    -arm_length * CT(1) / sqrt(2),  arm_length * CT(2) / sqrt(2),   arm_length * CT(3) / sqrt(2), -arm_length * CT(4) / sqrt(2);
    arm_length * CT(1) / sqrt(2),  -arm_length * CT(2) / sqrt(2),   arm_length * CT(3) / sqrt(2), -arm_length * CT(4) / sqrt(2);
    prop_diameter * CP(1) / (2 * pi),  prop_diameter * CP(2) / (2 * pi),   -prop_diameter * CP(3) / (2 * pi), -prop_diameter * CP(4) / (2 * pi);
    CT(1), CT(2), CT(3), CT(4)
];


% Transformation angular velocity to rate of change in Euler angles
Z = [1,    sin(eta(1))*tan(eta(2)),   cos(eta(1))*tan(eta(2));
     0,    cos(eta(1)),              -sin(eta(1));
     0,    sin(eta(1))/cos(eta(2)),   cos(eta(1))/cos(eta(2))];

% Transformation from body frame to inertial frame
R =  [cos(eta(2))*cos(eta(3)),   sin(eta(1))*sin(eta(2))*cos(eta(3))-cos(eta(1))*sin(eta(3)), cos(eta(1))*sin(eta(2))*cos(eta(3))+sin(eta(1))*sin(eta(3));
      cos(eta(2))*sin(eta(3)),   sin(eta(1))*sin(eta(2))*sin(eta(3))+cos(eta(1))*cos(eta(3)), cos(eta(1))*sin(eta(2))*sin(eta(3))-sin(eta(1))*cos(eta(3));
      -sin(eta(2)),              sin(eta(1))*cos(eta(2)),                                     cos(eta(1))*cos(eta(2))];
  
% Calculate resulting thrust and torque using B matrix
output = [ air_density * prop_diameter^4 * B(1,:) * rpm_square;
           air_density * prop_diameter^4 * B(2,:) * rpm_square;
           air_density * prop_diameter^4 * B(3,:) * rpm_square;
           air_density * prop_diameter^4 * B(4,:) * rpm_square ];

torque = output(1:3);
thrust = output(4);

eta_dot = Z * omega;
omega_dot = J^-1 * (cross(J * omega, omega) + torque);

pos_dot = R * v;

thrust_body = [0; 0; thrust];
g_body = R^-1 * g;
v_dot = thrust_body / mass + g_body - cross(omega, v);

end
