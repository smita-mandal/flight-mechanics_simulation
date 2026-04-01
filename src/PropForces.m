function [F_t] = PropForces(X,U,FlightData,rho)
% Constants
    P_max =  FlightData.Prop.P_max;    % propeller max power at sea level
    etaP = FlightData.Prop.eta;        % propeller efficiency
    deltaT = U(1);                     % throttle value
    rho0 = 1.2256;                     % Sea Level density (kg/m^3)
    a = 1.1324;                        % Propeller Parameters
    b = 0.1324;                        % Propeller Parameters
    
    % Extract values
    u = X(1);
    v = X(2);
    w = X(3);
    
    % Compute Velocity magnitude
    V = sqrt((u^2)+(v^2)+(w^2));
    
    % Compute Density Ratio
    sigma = rho/rho0;
    
    % Compute Thrust
    P_max_flight = P_max * ((a*sigma)-b);
    F_t = (etaP * P_max_flight *deltaT)/V;

end