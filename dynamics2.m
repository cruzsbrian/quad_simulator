function [phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot]= dynamics2(pwm_1, pwm_2, pwm_3, pwm_4, phi, theta, psi, p, q, r)
%#codegen

linear_tangent = 0.1716;
linear_intercept = 804.3572;

m = 1.462;   % Complete mass (kg)
g = 9.81;

Ixx = 1.0181101*10^(-2);   % Moments (kg*m^2)
Iyy = 0.97079192*10^(-2);
Izz = 1.8834745*10^(-2);

arm_length	= 0.2667;
air_density		= 1.225;
propeller_diameter = 0.254;

% Use a polynomial approximation to estimate power and thrust from each
% prop based on its speed
CP = 0.063811 + (2.006e-06)*[pwm_1, pwm_2, pwm_3, pwm_4] - (1.8024e-10)*[pwm_1^2, pwm_2^2, pwm_3^2, pwm_4^2];
CT =  0.13596 + (5.5549e-06)*[pwm_1, pwm_2, pwm_3, pwm_4] - (9.4623e-11)*[pwm_1^2, pwm_2^2, pwm_3^2, pwm_4^2];

% Aerodynamic coefficient matrix based on the above values
B = [- arm_length*CT(1)/sqrt(2),          arm_length*CT(2)/sqrt(2),          arm_length*CT(3)/sqrt(2),        - arm_length*CT(4)/sqrt(2);
    arm_length*CT(1)/sqrt(2),        - arm_length*CT(2)/sqrt(2),          arm_length*CT(3)/sqrt(2),        - arm_length*CT(4)/sqrt(2);
    propeller_diameter*CT(1)/(2*pi),   propeller_diameter*CT(2)/(2*pi), - propeller_diameter*CT(3)/(2*pi), - propeller_diameter*CT(4)/(2*pi);
    CT(1),                             CT(2),                             CT(3),                             CT(4)];

disp('Correct B');
disp(B);

eta=[phi; theta; psi];
omega = [p; q; r];

% Transformation from body frame to inertial frame, same as Z in
% mc_att_control
A2 = [1,    sin(eta(1))*tan(eta(2)),   cos(eta(1))*tan(eta(2));
      0,    cos(eta(1)),              -sin(eta(1));
      0,    sin(eta(1))/cos(eta(2)),   cos(eta(1))/cos(eta(2))];

% Limit pwm to acheivable values
real_pwm = min([pwm_1; pwm_2; pwm_3; pwm_4], 1950);
real_pwm = max(real_pwm, 1230);

% Approximate motor speed from linear fit with pwm
rpm = (real_pwm - linear_intercept)/linear_tangent;
rpm_square = rpm.^2;

% Calculate resulting thrust and torque using B matrix
output = (air_density * propeller_diameter^4) * (B*rpm_square);

torque = output(1:3)';
thrust = output(4);

eta_dot = A2*omega;

phi_dot = eta_dot(1);
theta_dot= eta_dot(2);
psi_dot =eta_dot(3);

p_dot = ((Iyy-Izz)*omega(2)*omega(3) + torque(1))/Ixx;
q_dot = ((Izz-Ixx)*omega(1)*omega(3) + torque(2))/Iyy;
r_dot = ((Ixx-Iyy)*omega(1)*omega(2) + torque(3))/Izz;
end
