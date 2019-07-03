function [eta_dot, omega_dot]= dynamics(pwm, eta, omega)

% Physical constants
g = 9.81;
air_density = 1.225;
m = 1.462;
Ixx = 1.0181101*10^(-2);
Iyy = 0.97079192*10^(-2);
Izz = 1.8834745*10^(-2);
arm_length = 0.266;
prop_diameter = 0.254;

% Estimate power and thrust coefficients using polynomial approximation
CP = 0.063811 + (2.006e-6)*pwm - (1.8024e-10)*pwm.^2;
CT = 0.135960 + (5.5549-6)*pwm - (9.4623e-10)*pwm.^2;

% Aerodynamic coefficient matrix from above values
B = [
    -arm_length * CT(1) / sqrt(2),  arm_length * CT(2) / sqrt(2),   arm_length * CT(3) / sqrt(2), -arm_length * CT(4) / sqrt(2);
    arm_length * CT(1) / sqrt(2),  -arm_length * CT(2) / sqrt(2),   arm_length * CT(3) / sqrt(2), -arm_length * CT(4) / sqrt(2);
    prop_diameter * CT(1) / (2 * pi),  prop_diameter * CT(2) / (2 * pi),   -prop_diameter * CT(3) / (2 * pi), -prop_diameter * CT(4) / (2 * pi);
    CT(1), CT(2), CT(3), CT(4)
];

% Transformation from body frame to intertial frame
Z = [1,    sin(eta(1))*tan(eta(2)),   cos(eta(1))*tan(eta(2));
     0,    cos(eta(1)),              -sin(eta(1));
     0,    sin(eta(1))/cos(eta(2)),   cos(eta(1))/cos(eta(2))];

% Limit pwm to acheivable values
real_pwm = max(min(pwm, 1950), 1230);

% Approximate motor speed from linear fit with pwm

rpm = (real_pwm - 0.1716) / 804.3572;
rpm_square = rpm.^2;

% Calculate resulting thrust and torque using B matrix
output = air_density * prop_diameter^4 * B * rpm_square;

torque = output(1:3)';
thrust = output(4);

eta_dot = Z * omega;

omega_dot = [
    ((Iyy - Izz) * omega(2) * omega(3) + torque(1)) / Ixx;
    ((Izz - Ixx) * omega(1) * omega(3) + torque(2)) / Iyy;
    ((Ixx - Iyy) * omega(1) * omega(2) + torque(3)) / Izz;
];
