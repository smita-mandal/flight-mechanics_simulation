function [Xbardot0]= Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0,V,alpha,beta,gamma,rho,Qbar,CL,perturbation)

    dT = U0(1);
    de = U0(2);
    da = U0(3);
    dr = U0(4); 
    % Inertial Coefficients
    Ixx = FlightData.Inertial.Ixx;
    Iyy = FlightData.Inertial.Iyy;
    Izz = FlightData.Inertial.Izz;
    Ixz = FlightData.Inertial.Ixz;
    C0 = Ixx*Izz - Ixz^2;
    C1 = Izz/C0;
    C2 = Ixz/C0;
    C3 = C2*(Ixx-Iyy+Izz);
    C4 = (C1*(Iyy-Izz))-(C2*Ixz);
    C5 = 1/Iyy;
    C6 = C5*Ixz;
    C7 = C5*(Izz-Ixx);
    C8 = Ixx/C0;
    C9 = (C8*(Ixx-Iyy))+(C2*Ixz);

    % Convert Quaternions to Euler Angles
    quat = [Xk0(7);Xk0(8);Xk0(9);Xk0(10)];
    [euler] = q2e(quat);     
    phi = euler(1);

    nz = 1/cos(phi);
    
    % Update pitch angle as per constraint
    theta = alpha + (gamma * (pi/180));
    

    if ((strcmp(perturbation, 'alpha')) || strcmp(perturbation, 'beta'))

        % Calculate initial velocity components
        u = V * cos(alpha)*cos(beta);
        v = V * sin(beta);
        w = V * sin(alpha)*cos(beta);
    
        % Update pitch angle as per constraint
        theta = alpha + (gamma * (pi/180));
    
        % Compute airspeed (V) magnitude
        V = sqrt(u^2 + v^2 + w^2);        
        
        % Body rate  
        
        q = (FlightData.Inertial.g * (nz - (1/nz)))/V; 
        p = Xk0(4);
        r = Xk0(6);
        
        if ((strcmp(perturbation, 'alpha')))
        if (((phi == 0) || (mod((phi/(pi/2)),2)==0)))
            r = 0;
            
        else
            % Calculate body rate, r
            R = (V^2)/(FlightData.Inertial.g* tan(phi)); % Radius of turn
            psidot = V/R; 
            thetadot = (q - (cos(theta)*sin(phi)*psidot))/cos(phi);             
            r = (-sin(phi)*thetadot)+ ((cos(theta)*cos(phi))*psidot);
            
        end
        end        
        % Calculate State Rates
        udot = (r*v)-(q*w);
        vdot = (-r*u)+(p*w);
        wdot = (q*u)-(p*v);
        
        pdot = (C3*p*q)+(C4*q*r);
        qdot = (C7*p*r)-(C6*((p^2)-(r^2)));
        rdot = (C9*p*q)-(C3*q*r);
        
        if (Xbar_size==6)
            Xbardot0 = [udot,vdot,wdot,pdot,qdot,rdot]';
        elseif (Xbar_size==5)
            Xbardot0 = [udot,wdot,pdot,qdot,rdot]';
        else
            Xbardot0 = [udot,wdot,qdot]';
        end

    elseif (strcmp(perturbation, 'dT'))        
        
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
        CD = FlightData.Aero.Cdo + (FlightData.Aero.k*(CL^2));
        
        CX = (-CD * cos(alpha)) + ((-CL)*(-sin(alpha)));  
        
        F_t = (CX .* (Qbar * FlightData.Geo.S)); %for equilibrium flight        

        V = (dT * (etaP * P_max_flight))/F_t;

        % Calculate initial velocity components
        u = V * cos(alpha)*cos(beta);
        v = V * sin(beta);
        w = V * sin(alpha)*cos(beta);

        q = (FlightData.Inertial.g * (nz - (1/nz)))/V;
        p = Xk0(4);
        if (((phi == 0) || (mod((phi/(pi/2)),2)==0)))

                r = 0;
                
        else
                % Calculate body rate, r
                R = (V^2)/(FlightData.Inertial.g* tan(phi)); % Radius of turn
                psidot = V/R; 
                thetadot = (q - (cos(theta)*sin(phi)*psidot))/cos(phi);             
                r = (-sin(phi)*thetadot)+ ((cos(theta)*cos(phi))*psidot);                
        end

        % Calculate State Rates
        udot = (r*v)-(q*w);
        vdot = (-r*u)+(p*w);
        wdot = (q*u)-(p*v);

        pdot = (C3*p*q)+(C4*q*r);
        qdot = (C7*p*r)-(C6*((p^2)-(r^2)));
        rdot = (C9*p*q)-(C3*q*r);
        
        if (Xbar_size==6)
            Xbardot0 = [udot,vdot,wdot,pdot,qdot,rdot]';
        elseif (Xbar_size==5)
            Xbardot0 = [udot,wdot,pdot,qdot,rdot]';
        else
            Xbardot0 = [udot,wdot,qdot]';
        end

    elseif (strcmp(perturbation, 'de'))
        u = Xk0(1);
        v = Xk0(2); 
        w = Xk0(3); 
        p = Xk0(4);
        q = Xk0(5); 
        r = Xk0(6); 

        pdot = Xkdot0(4);
        rdot = Xkdot0(6);

        %Find change in velocity with time     
        udot = (r*v)-(q*w);
        vdot = (-r*u)+(p*w);
        wdot = (q*u)-(p*v);

        % Compute differential velocity
        Vdot = ((u*udot) + (v*vdot) + (w*wdot))/V;    
        
        % Compute aerodynamic angle differentials        
        alphadot = ((u*wdot) - (w*udot))/((u^2)+(w^2));
               
        % Non-dimensional Angular Rates and body rates
        alphadotbar = (alphadot*FlightData.Geo.c)/(2*V);       
        qbar = (q*FlightData.Geo.c)/(2*V);
        
        Cm = FlightData.Aero.Cmo + (FlightData.Aero.Cma * alpha) + (FlightData.Aero.Cmad * alphadotbar) + (FlightData.Aero.Cmq * qbar) + (FlightData.Aero.Cmde*de);

        M = Cm*FlightData.Geo.c*Qbar*FlightData.Geo.S;

        qdot = (C7*p*r)-(C6*((p^2)-(r^2)))+(C5*M);

        if (Xbar_size==6)
            Xbardot0 = [udot,vdot,wdot,pdot,qdot,rdot]';
        elseif (Xbar_size==5)
            Xbardot0 = [udot,wdot,pdot,qdot,rdot]';
        else
            Xbardot0 = [udot,wdot,qdot]';
        end

    elseif ((strcmp(perturbation, 'da')) || strcmp(perturbation, 'dr'))
        u = Xk0(1);
        v = Xk0(2); 
        w = Xk0(3); 
        p = Xk0(4);
        q = Xk0(5); 
        r = Xk0(6); 

        qdot = Xkdot0(5);        

        %Find change in velocity with time     
        udot = (r*v)-(q*w);
        vdot = (-r*u)+(p*w);
        wdot = (q*u)-(p*v);

        % Compute differential velocity
        Vdot = ((u*udot) + (v*vdot) + (w*wdot))/V;    
        
        % Compute aerodynamic angle differentials        
        betadot = (vdot/V) - ((v*Vdot)/(V^2))/sqrt(1-((v^2)/(V^2)));        
        
        % Non-dimensional Angular Rates and body rates        
        betadotbar = (betadot*FlightData.Geo.b)/(2*V);
        pbar = (p*FlightData.Geo.b)/(2*V);
        rbar = (r*FlightData.Geo.b)/(2*V);   

        Cl = FlightData.Aero.Clb*beta + FlightData.Aero.Clbd*betadotbar + FlightData.Aero.Clr*rbar + FlightData.Aero.Clp*pbar + FlightData.Aero.Clda*da + FlightData.Aero.Cldr*dr;
        Cn = FlightData.Aero.Cnb*beta + FlightData.Aero.Cnbd*betadotbar + FlightData.Aero.Cnr*rbar + FlightData.Aero.Cnp*pbar + FlightData.Aero.Cnda*da + FlightData.Aero.Cndr*dr;
    
        L = Cl*FlightData.Geo.b*Qbar*FlightData.Geo.S;
        N = Cn*FlightData.Geo.b*Qbar*FlightData.Geo.S;

        pdot = (C3*p*q)+(C4*q*r)+(C1*L)+(C2*N);       
        rdot = (C9*p*q)-(C3*q*r)+(C2*L)+(C8*N);

        if (Xbar_size==6)
            Xbardot0 = [udot,vdot,wdot,pdot,qdot,rdot]';
        elseif (Xbar_size==5)
            Xbardot0 = [udot,wdot,pdot,qdot,rdot]';
        else
            Xbardot0 = [udot,wdot,qdot]';
        end

    end    
end

        
        
          