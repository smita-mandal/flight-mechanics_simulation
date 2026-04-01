function [alpha,beta,V,U]= Trim_control_inputs(trim_case,FlightData,rho,Qbar,nz,Xbar,Xdot,Xbar_size,CL)
      
     if (Xbar_size==6)
         u = Xbar(1);
         v = Xbar(2);
         w = Xbar(3);
         p = Xbar(4);
         q = Xbar(5);
         r = Xbar(6);

     elseif (Xbar_size==3)
         u = Xbar(1);
         v = 0;
         w = Xbar(2); 
         p = 0;
         q = Xbar(3);
         r = 0;
     else
         disp("Incorrect input");
     end   
     
     
    % Compute airspeed (V) magnitude
    V = sqrt(u^2 + v^2 + w^2);

    % Compute the angle of attack (alpha) in radians
    alpha = atan(w/u); 

    % Compute the side-slip angle (beta) in radians
    beta = asin(v / V);
    
    % --------------------------------------------
    % Calculate elevator deflection
    % --------------------------------------------
    udot = (r*v) - (q*w);
    vdot = (p*w) - (r*u);
    wdot = (q*u) - (p*v);
    % Compute differential velocity
    Vdot = ((u*udot) + (v*vdot) + (w*wdot))/V;    
    
    % Compute aerodynamic angle differentials        
    alphadot = ((u*wdot) - (w*udot))/((u^2)+(w^2));
    betadot = (vdot/V) - ((v*Vdot)/(V^2))/sqrt(1-((v^2)/(V^2)));  
    
    
    % Non-dimensional Angular Rates and body rates
    alphadotbar = (alphadot*FlightData.Geo.c)/(2*V);
    betadotbar = (betadot*FlightData.Geo.b)/(2*V);
    pbar = (p*FlightData.Geo.b)/(2*V);
    qbar = (q*FlightData.Geo.c)/(2*V);
    rbar = (r*FlightData.Geo.b)/(2*V);
    
    % Compute Lift and Drag coefficients from differential elements  
    if strcmp(trim_case, 'steady_mnvr')
        de = (-FlightData.Aero.Cmo-(FlightData.Aero.Cma*alpha)-(FlightData.Aero.Cmad*alphadotbar)-(FlightData.Aero.Cmq*qbar))/FlightData.Aero.Cmde
        CL = FlightData.Aero.CLo + (FlightData.Aero.CLa * alpha) + (FlightData.Aero.CLad * alphadotbar) + (FlightData.Aero.CLq * qbar) + (FlightData.Aero.CLde*de)

    elseif strcmp(trim_case, 'steady_level')
        de = (CL - FlightData.Aero.CLo+(FlightData.Aero.CLa * alpha)-(FlightData.Aero.CLad*alphadotbar)-(FlightData.Aero.CLq*qbar))/FlightData.Aero.CLde; 
    end
        % --------------------------------------------
    % Calculate aileron and rudder deflection
    % --------------------------------------------    
    RR1 = -(FlightData.Aero.Clb*beta)-(FlightData.Aero.Clbd*betadotbar) - (FlightData.Aero.Clr*rbar) - (FlightData.Aero.Clp*pbar);
    RR2 = -(FlightData.Aero.Cnb*beta)-(FlightData.Aero.Cnbd*betadotbar) - (FlightData.Aero.Cnr*rbar) - (FlightData.Aero.Cnp*pbar) ;
    R = [RR1;RR2];

    LR1 = [FlightData.Aero.Clda,FlightData.Aero.Cldr];
    LR2 = [FlightData.Aero.Cnda,FlightData.Aero.Cndr];
    L = [LR1;LR2];
    
    Z = inv(L) * R;
    
    da = Z(1)
    dr = Z(2)

    % --------------------------------------------
    % Calculate Thrust Factor deflection
    % --------------------------------------------        
    %Cy = (FlightData.Aero.Cyb*beta)+(FlightData.Aero.Cybd*betadotbar)+(FlightData.Aero.Cyr*rbar)+(FlightData.Aero.Cyp*pbar)+(FlightData.Aero.Cyda*da)+(FlightData.Aero.Cydr*dr);
    Cy = (FlightData.Inertial.m * FlightData.Inertial.g)/(Qbar * FlightData.Geo.S);
    % Calculate Thrust Setting    
    P_max =  FlightData.Prop.P_max;    % propeller max power at sea level
    etaP = FlightData.Prop.eta;        % propeller efficiency    
    rho0 = 1.2256;                     % Sea Level density (kg/m^3)
    a = 1.1324;                        % Propeller Parameters
    b = 0.1324;                        % Propeller Parameters
    % Compute Density Ratio
    sigma = rho/rho0;    
    % Compute Thrust
    P_max_flight = P_max * ((a*sigma)-b);

    %Convert into Body
    Cbs = C_y(alpha);    
    CD = FlightData.Aero.Cdo + (FlightData.Aero.k*(CL^2));

    body_coeffs = Cbs * [-CD;Cy;-CL];
    CX = body_coeffs(1);    
    
    F_t = (CX .* (Qbar * FlightData.Geo.S)); %for equilibrium flight

    dT = (F_t*V)/(etaP * P_max_flight)
    % --------------------------------------------

    U = [dT,de,da,dr]';   
end

        
        
          