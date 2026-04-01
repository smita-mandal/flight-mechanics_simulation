function [rho, Qbar] = FlowProperties(X)

    % Declare constants
    L = -0.0065;        % lapse rate (K/m)
    g = 9.81;           % acceleration due to gravity (m/s^2)
    R = 287.05;         % gas constant for dry air (J/kg/K)
    P0 = 101325;        % sea-level static pressure (Pa)
    T0 = 288.15;        % sea-level static temperature (K)
    
    % Extract altitude from state vector
    [rownum, colnum] = size(X);
    if (rownum==12 && colnum==1)
        h = -X(12);         % altitude (m)
    elseif (rownum==13 && colnum==1)
        h = -X(13);         % altitude (m)
    else
        disp('Incorrect size of State Vector');
    end

    % Extract velocity components in x, y and z axes
    u = X(1);           
    v = X(2);
    w = X(3);
    
    expo = -g/(R*L);
    
    % Set to default pressure, temperature and density, if altitude is below
    % sea-level
    if (h < 0)
        rho = 1.2256;                            % Standard Sea level density (kg/m^3)                
        disp('Warning: Altitude is below sea level');
    else
        % Compute Atomspheric Conditions; T, P, rho
        T = T0 + (L*h);                          % Static Temperature (K)
        P = P0 * ((1 + ((L*h)/T0))^expo);        % Static Pressure (Pa)
        rho = P/(R*T);                          % Density (kg/m^3)
    end
    % Compute Velocity Magnitude  
    V = sqrt((u^2) + (v^2) + (w^2));  
    
    % Compute Dynamic Pressure, Qbar (Pa)
    Qbar =  (1/2) * rho * (V^2);   
    
end
