function [pos_dot, v_dot, eta_dot, omega_dot]= dynamics(pwm, pos, v, eta, omega)

% Physical constants
air_density = 1.225;

mass = 1.462;
g = [0; 0; -9.81];

Ixx = 1.0181101*10^(-2);
Iyy = 0.97079192*10^(-2);
Izz = 1.8834745*10^(-2);
J = [ Ixx, 0, 0;
      0, Iyy, 0;
      0, 0, Izz ];

arm_length = 0.2667;
prop_diameter = 0.254;

% Estimate power and thrust coefficients using polynomial approximation
CP = 0.063811 + (2.006e-6)*pwm - (1.8024e-10)*pwm.^2;
CT = 0.135960 + (5.5549e-6)*pwm - (9.4623e-11)*pwm.^2;

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
  

% Limit pwm to acheivable values
real_pwm = max(min(pwm, 1950), 1230);

% Approximate motor speed from linear fit with pwm
rpm = (real_pwm - 804.3572) / 0.1716;
rpm_square = rpm.^2;

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
