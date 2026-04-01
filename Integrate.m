function [Xn,Xdotn] = Integrate(FlightData,X0,Xdot0,U0,time,dt)
    
    Xdotn = AngularRateConvergence(FlightData,X0,Xdot0,U0,time,dt);
       

    % Calculate average gradient for the time step
    Xdot1 = Xdotn;
    An = Xdot1 .* dt;
    
    Xdot2 = Xdotn + (An./2);
    Bn = Xdot2 .* dt;
    
    Xdot3 = Xdotn + (Bn./2);
    Cn = Xdot3 .* dt;
    
    Xdot4 = Xdotn + (Cn);
    Dn = Xdot4 .* dt;
    
    % Calculate State vector for the time step
    Xn = X0 + ((1/6).*(An + (2.*Bn) + (2.*Cn) + Dn)) ;   
end




