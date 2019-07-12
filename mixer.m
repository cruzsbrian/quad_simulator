function [rpm] = mixer(control_thrust, control_torque)

% Physical constants
prop_diameter = 0.045;
air_density   = 1.225;
arm_length = 0.066;

% Approximate CP and CT as constant, using data collected in Forster's thesis
CT = 4.2e-5;
CP = 3.8e-5;

% Aerodynamic coefficient matrix from above values
B = [
    -arm_length * CT / sqrt(2),  arm_length * CT / sqrt(2),   arm_length * CT / sqrt(2), -arm_length * CT / sqrt(2);
    arm_length * CT / sqrt(2),  -arm_length * CT / sqrt(2),   arm_length * CT / sqrt(2), -arm_length * CT / sqrt(2);
    prop_diameter * CP / (2 * pi),  prop_diameter * CP / (2 * pi),   -prop_diameter * CP / (2 * pi), -prop_diameter * CP / (2 * pi);
    CT, CT, CT, CT
];

input = [control_torque; control_thrust];

% Note that this is different from the equation used in the paper
% It is taken from the attitude_mode simulink file
%rpm_square = invB0*(input - (air_density * prop_diameter^4)* B1*[1; 1; 1; 1]);

% Use inverse of B matrix to find propeller speeds
rpm_square = (1 / (air_density * prop_diameter^4)) * B^-1 * input;
rpm_square = max(rpm_square,0);
rpm = sqrt(rpm_square);
end
