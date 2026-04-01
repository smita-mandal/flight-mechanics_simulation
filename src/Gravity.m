function g_body = Gravity(X,g)
%Find transformation for earth to body

    % Compute gravitational acceleration components in Earth-axis and Body-axis
    % g: Gravitational acceleration (scalar, e.g., 9.81 m/s^2)
    % X: Aircraft state vector

    % Call the DCM function to get the transformation matrix C_be
    C_be = DCM(X);

    % Calculate gravitational acceleration in Earth-axis
    g_earth = [0; 0; g];

    %Convert Gravity into body axis
    g_body = C_be * g_earth;

end
