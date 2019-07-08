function [pwm] = mixer(control_thrust, control_torque)

% Physical constants
prop_diameter = 0.2667;
air_density   = 1.225;

% This uses a linear approximation of the aerodynamic coefficient matrix
invB0 = 1.0e+04 * [
   -0.1598,    0.1598,    1.5335,    0.0301;
    0.1598,   -0.1598,    1.5335,    0.0301;
    0.1598,    0.1598,   -1.5335,    0.0301;
   -0.1598,   -0.1598,   -1.5335,    0.0301];
     
B1 = 1.0e+04 * [    
    1.3776,   -1.3776,   -1.3776,    1.3776;
   -1.3776,    1.3776,   -1.3776,    1.3776;
   -0.1985,   -0.1985,    0.1985,    0.1985;
   -7.3047,   -7.3047,   -7.3047,   -7.3047];

input = [control_torque; control_thrust];

% Note that this is different from the equation used in the paper
% It is taken from the attitude_mode simulink file
rpm_square = invB0*(input - (air_density * prop_diameter^4)* B1*[1; 1; 1; 1]);
rpm_square = max(rpm_square,0);
rpm = sqrt(rpm_square);

% Use a linear fit to find approximate pwm to get this motor speed
pwm = 0.1716 * rpm + 804.3572;
end

