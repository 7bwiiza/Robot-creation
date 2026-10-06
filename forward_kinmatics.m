
%theta
theta1 = 30;
theta2 = 45;
theta3 = 10;
theta4 = 0;

function T = forward_kinematics(theta1, theta2, theta3, theta4)

    % Convert degrees to radians
    theta1 = deg2rad(theta1);
    theta2 = deg2rad(theta2);
    theta3 = deg2rad(theta3);
    theta4 = deg2rad(theta4);

    % Link parameters
    d1 = 75; a2 = 100; a3 = 60; a4 = 35;
    alpha1 = pi/2;

    % Transformation Matrices
    T01 = dh_transform(theta1, d1, 0, alpha1)
    T12 = dh_transform(theta2, 0, a2, 0)
    T23 = dh_transform(theta3, 0, a3, 0)
    T34 = dh_transform(theta4, 0, a4, 0)

    % Final Transformation
    T = T01 * T12 * T23 * T34
end

function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0          sin(alpha)             cos(alpha)            d;
         0          0                      0                     1];
end
