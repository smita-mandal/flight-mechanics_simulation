function [alpha, beta] = AeroAngles(X)
%Calculate wind angles based on current state X
    % Parse the aircraft state vector
    u = X(1);  % Body-relative velocity along x-axis (m/s)
    v = X(2);  % Body-relative velocity along y-axis (m/s)
    w = X(3);  % Body-relative velocity along z-axis (m/s)

    % Compute airspeed (V) magnitude
    V = sqrt(u^2 + v^2 + w^2);

    % Compute the angle of attack (alpha) in radians
    alpha = atan(w/u);

    % Compute the side-slip angle (beta) in radians
    beta = asin(v / V);
end
