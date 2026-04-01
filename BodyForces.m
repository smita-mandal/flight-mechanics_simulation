function [Forces, Moments] = BodyForces(X,Xdot,U,FlightData,time)
    % Compute the [3x1] forces and [3x1] moments on the aircraft for a given state
    u = X(1);   % Body-relative velocity along x-axis (m/s)
    v = X(2);   % Body-relative velocity along y-axis (m/s)
    w = X(3);   % Body-relative velocity along z-axis (m/s
    p = X(4);   %Roll angle
    q = X(5);   %Pitch angle
    r = X(6);   %Yaw angle
    de = U(2);  %Elevator Deflection
    da = U(3);  %Aileron Delfection
    dr = U(4);  %Rudder Deflection
    
    Forces = zeros(3,1);    %Matrix for Forces
    Moments = zeros(3,1);   %Matrix for Moments
    
    % Compute Velocity Magnitude
    V = sqrt(u^2+v^2+w^2);
    
    % Compute aerodynamic angles and rates
    [alpha, beta] = AeroAngles(X);
    [alphadot, betadot] = AngularRates(X,Xdot,time);
    
    % Calculate dynamic pressure and density
    [rho, Qbar] = FlowProperties(X);

    % Non-dimensional Angular Rates and body rates
    alphadotbar = (alphadot*FlightData.Geo.c)/(2*V);
    betadotbar = (betadot*FlightData.Geo.b)/(2*V);
    pbar = (p*FlightData.Geo.b)/(2*V);
    qbar = (q*FlightData.Geo.c)/(2*V);
    rbar = (r*FlightData.Geo.b)/(2*V);
    
     % Calulate Wind Forces
    [Lift, Drag] = WindForces(X,Xdot,U,FlightData,time);   

    %Longitudnal Coefficients
    CD = Drag / (Qbar * FlightData.Geo.S);
    CL = Lift / (Qbar * FlightData.Geo.S);
    Cm = FlightData.Aero.Cmo + (FlightData.Aero.Cma * alpha) + (FlightData.Aero.Cmad * alphadotbar) + (FlightData.Aero.Cmq * qbar) + (FlightData.Aero.Cmde*de); 
    %Lateral-Directional Coefficients
    Cy = (FlightData.Aero.Cyb*beta) + (FlightData.Aero.Cybd*betadotbar) + (FlightData.Aero.Cyr*rbar) + (FlightData.Aero.Cyp*pbar) + (FlightData.Aero.Cyda*da) + (FlightData.Aero.Cydr*dr);
    Cl = FlightData.Aero.Clb*beta + FlightData.Aero.Clbd*betadotbar + FlightData.Aero.Clr*rbar + FlightData.Aero.Clp*pbar + FlightData.Aero.Clda*da + FlightData.Aero.Cldr*dr;
    Cn = FlightData.Aero.Cnb*beta + FlightData.Aero.Cnbd*betadotbar + FlightData.Aero.Cnr*rbar + FlightData.Aero.Cnp*pbar + FlightData.Aero.Cnda*da + FlightData.Aero.Cndr*dr;
    
    % Calculate Thrust Force
    F_t = PropForces(X,U, FlightData,rho);
    
    %Convert into Body
    Cbs = C_y(alpha);
    
    body_coeffs = Cbs * [-CD;Cy;-CL];
    CX = body_coeffs(1);
    CY = body_coeffs(2);
    CZ = body_coeffs(3);
       
    % Calculate Body Forces;
    FX = (CX .* (Qbar * FlightData.Geo.S)) + F_t;
    FY = CY .* (Qbar * FlightData.Geo.S);
    FZ = CZ .* (Qbar * FlightData.Geo.S);

    % Calculate Moments
    L = Cl*FlightData.Geo.b*Qbar*FlightData.Geo.S;
    M = Cm*FlightData.Geo.c*Qbar*FlightData.Geo.S;
    N = Cn*FlightData.Geo.b*Qbar*FlightData.Geo.S;

    % Collect forces and moments into [3x1] arrays respectively
    Forces(1) = FX;
    Forces(2) = FY;
    Forces(3) = FZ;
    
    Moments(1) = L;
    Moments(2) = M;
    Moments(3) = N;

end
