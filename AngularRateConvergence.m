function [Xdot] = AngularRateConvergence(FlightData,X0,Xdot0,U0,time,dt)
        
        if (time ==0) 
            Xdot0 = zeros(13,1);            
            % Initial values of angular rates
            alphadotinit = 0;
            betadotinit = 0;
            % Initial State Rate Vector
            Xdotinit = StateRates(X0,Xdot0,U0,FlightData,time);
            
            % Calculate Angular Rate using initial State Rate vector
            [alphadotf, betadotf] = AngularRates(X0,Xdotinit,time+dt);
            
            
            % Iterate to determine the State Rate vector 
            while ((alphadotinit~=alphadotf) | (betadotinit~=betadotf))
                Xdot = StateRates(X0,Xdotinit,U0,FlightData,time);
                alphadotinit = alphadotf;
                betadotinit = betadotf;
                [alphadotf, betadotf] = AngularRates(X0,Xdot,time+dt); 
                Xdotinit = Xdot; 
                
            end
            
        else
            Xdot = StateRates(X0,Xdot0,U0,FlightData,time);
            
        end
           
end