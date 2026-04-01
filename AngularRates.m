function [alphadot, betadot] = AngularRates(X,Xdot,time)
    %Calculate anglur rates based on current state X and the rates.
    
    if (time ==0) 
        % Initialize to zero at initial time step, since the values are
        % unknown
        alphadot = 0;
        betadot = 0;
    else   
        % Calculate angular rates based on state and state rate vector

        u = X(1);             % Body-relative velocity along x-axis (m/s)
        v = X(2);             % Body-relative velocity along y-axis (m/s)
        w = X(3);             % Body-relative velocity along z-axis (m/s)
        udot = Xdot(1);       % Body-relative velocity rate along x-axis
        vdot = Xdot(2);       % Body-relative velocity rate along y-axis  
        wdot = Xdot(3);       % Body-relative velocity rate along z-axis
        
        % Compute velocity magnitude
        V = sqrt(u^2 + v^2 + w^2);

        % Compute differential velocity
        Vdot = ((u*udot) + (v*vdot) + (w*wdot))/V;    
        
        % Compute aerodynamic angle differentials        
        alphadot = ((u*wdot) - (w*udot))/((u^2)+(w^2));
        betadot = (vdot/V) - ((v*Vdot)/(V^2))/sqrt(1-((v^2)/(V^2)));        
    end

end
