function [Lift, Drag] = WindForces(X,Xdot,U,FlightData,time)
% Compute the aerodynamic Forces on the Aircraft due to the current state vectors
    % Extract the values
    de = U(2); 
    u = X(1);
    v = X(2);
    w = X(3);
    q = X(5);
    % Compute velocity Magnitude
    V = sqrt((u^2)+(v^2)+(w^2));
    
    % Get Aerodynamic Angles and rates
    [alpha, beta] = AeroAngles(X);
    [alphadot, betadot] = AngularRates(X,Xdot,time);
    
    % Get flow Properties
    [rho, Qbar] = FlowProperties(X);
    
    % non-dimensionalise terms
    alphadotbar = (alphadot * FlightData.Geo.c)/(2*V);
    qbar = (q * FlightData.Geo.c)/(2*V);
    
    % Compute Lift and Drag coefficients from differential elements
    CL = FlightData.Aero.CLo + (FlightData.Aero.CLa * alpha) + (FlightData.Aero.CLad * alphadotbar) + (FlightData.Aero.CLq * qbar) + (FlightData.Aero.CLde*de);
    Cd = FlightData.Aero.Cdo + (FlightData.Aero.k*(CL^2));
    
    % Compute Lift and Drag
    Lift = CL * Qbar * FlightData.Geo.S;
    Drag = Cd * Qbar * FlightData.Geo.S;
return