function [quat] = e2q(euler)
% Input to the function, e2q: euler angles (in degrees)
% Output of the function, e2q: quaternions

% Verify if euler angles vector is a 3xn matrix
[rownum, colnum] = size(euler);

if (~(rownum==3) && (colnum>=1))
    fprintf("Euler angles vector of aircraft should be a 3xn vector; Update the Euler angles")
end

% Euler Angles converted to radians
phi = (euler(1)).*(pi/180);
theta = (euler(2)).*(pi/180);
psi = (euler(3)).*(pi/180);

% Calculate quaternions
q0 = (cos(psi./2) .* cos(theta./2) .* cos(phi./2)) + (sin(psi./2) .* sin(theta./2) .* sin(phi./2));
q1 = (cos(psi./2) .* cos(theta./2) .* sin(phi./2)) - (sin(psi./2) .* sin(theta./2) .* cos(phi./2)); 
q2 = (cos(psi./2) .* sin(theta./2) .* cos(phi./2)) + (sin(psi./2) .* cos(theta./2) .* sin(phi./2)); 
q3 = ((-cos(psi./2)) .* sin(theta./2) .* sin(phi./2)) + (sin(psi./2) .* cos(theta./2) .* cos(phi./2)); 

quat = [q0;q1;q2;q3];
end
