function [theta1, theta2, theta3, theta4] = inverse_kinematics(px, py, pz)
    % Link lengths
    d1 = 75; a2 = 100; a3 = 60; a4 = 35;

    % Step 1: wrist center position
    wx = px - a4;  % assume the wrist points forward in x
    wy = py;
    wz = pz;

    % Step 2: Base angle
    theta1 = atan2(wy, wx);

    % Step 3: Wrist center in shoulder frame
    r = sqrt(wx^2 + wy^2);
    z = wz - d1;

    % Step 4: Use cosine law to find elbow angle
    D = (r^2 + z^2 - a2^2 - a3^2) / (2 * a2 * a3);
    theta3 = atan2(sqrt(1 - D^2), D); % elbow down

    % Step 5: Shoulder angle
    phi = atan2(z, r);
    psi = atan2(a3 * sin(theta3), a2 + a3 * cos(theta3));
    theta2 = phi - psi;

    % Step 6: Orientation – assuming flat wrist (no roll)
    theta4 = 0;  % or compute based on desired orientation

    % Convert to degrees
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);
    theta3 = rad2deg(theta3);
    theta4 = rad2deg(theta4);
end
