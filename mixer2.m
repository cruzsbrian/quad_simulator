function [pwm_1, pwm_2, pwm_3, pwm_4] = mixer2(thrust, control_1, control_2, control_3)
%#codegen
linear_tangent = 0.1716;
linear_intercept = 804.3572;

prop_diameter	= 0.2667;
air_density		= 1.225;

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

% control_1,2,3 is applied moment around each axis. thrust is overall
% thrust in the z axis
input = [control_1; control_2; control_3; thrust];

% Note that this is different from the equation used in the paper
rpm_square = invB0*(input - (air_density * prop_diameter^4)* B1*[1; 1; 1; 1]);
rpm_square = max(rpm_square,0);
rpm = sqrt(rpm_square);
pwm = linear_tangent*rpm + linear_intercept;
pwm_1 = pwm(1);
pwm_2 = pwm(2);
pwm_3 = pwm(3);
pwm_4 = pwm(4);
end
