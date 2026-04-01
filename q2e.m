function [euler] = q2e(quat)
% Input to the function, q2e: quaternions
% Output of the function, q2e: euler angles (in radians)

% Verify if quaternions vector is a 4xn matrix
[rownum, colnum] = size(quat);
if (~(rownum==4) && (colnum>=1))
    fprintf("Quaternions vector of aircraft should be a 4xn vector; Update the quaternions")
end

% Initialize quaternions values
q0 = quat(1);
q1 = quat(2);
q2 = quat(3);
q3 = quat(4);

% Calculate numerator and denominators w.r.t euler angles
phi_n = (q2.*q3) + (q0.*q1);
phi_d = ((q0.^2)+(q3.^2)-0.5);

theta_n= (q0.*q2) - (q1.*q3);
theta_d = sqrt((((q0.^2) + (q1.^2) - 0.5).^2) + (((q1.*q2) + (q0.*q3)).^2));

psi_n = (q1.*q2) + (q0.*q3);
psi_d = (q0.^2) + (q1.^2) - 0.5;

% Calculate Euler angles
phi = atan2(real(phi_d),real(phi_n));
theta = atan2(real(theta_d),real(theta_n));
psi = atan2(real(psi_d),real(psi_n));

euler = [phi;theta;psi];
euler = euler.*(180/pi);
end
