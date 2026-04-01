function [X0,Xbar0,alpha,beta,gamma,rho,Qbar,CL]= Trim_inputs(FlightData,Xbar_size,Xinit,V,trim_case)

     % Weight of the aircraft
     W = FlightData.Inertial.m * FlightData.Inertial.g;

     %Bank angle
     phi = Xinit(7) *(pi/180);

     %Yaw angle
     psi = Xinit(9) *(pi/180);

     % calculate normal load factor
     if ((phi==0) && (mod((phi/(pi/2)),2)~=0))         
          nz = 1;
     else
          nz = 1/cos(phi);
     end
    
     % Consider Climb angle
     gamma = 0;

     [rho, Qbar] = FlowProperties(Xinit);
     % Compute Dynamic Pressure, Qbar (Pa)
     Qbar =  (1/2) * rho * (V^2); 

     % Calculate lift coefficcient for level flight
     CL = (nz*W)/(0.5*rho*(V^2*FlightData.Geo.S));  % Lift coefficient at equilibrium

     if strcmp(trim_case, 'steady_mnvr_knifedge')
         % Update Lift as per bank angle
         CL = CL * cos(phi);
         % Calculate initial sideslip 
         beta = (nz * W)/(Qbar*FlightData.Geo.S*FlightData.Aero.Cyb);

     elseif strcmp(trim_case, 'steady_turn')
         % Update Lift as per bank angle
         CL = CL * cos(phi);
         % Initial sideslip 
         beta = 0;
     elseif strcmp(trim_case, 'steady_level')
         % Initial sideslip 
         beta = 0;
     else
         disp('Incorrect trim case')
     end
     
     % Initial angle of attack     
     alpha = (CL - FlightData.Aero.CLo)/FlightData.Aero.CLa; % radian  
   
     xe = Xinit(10);
     ye = Xinit(11);
     ze = Xinit(12);   
     
     % Calculate initial velocity components
     u = V * cos(alpha)*cos(beta);
     v = V * sin(beta);
     w = V * sin(alpha)*cos(beta);

     % Update pitch angle as per constraint
     theta = alpha + (gamma * (pi/180));
     
     % Initial Body rate
     
     q = (FlightData.Inertial.g * (nz - (1/nz)))/V;
     p = 0;

     if ((phi == 0) || (mod((phi/(pi/2)),2)~=0))
         r = 0;
         
     else
         % Calculate body rate, r
         R = (V^2)/(FlightData.Inertial.g* tan(phi)); % Radius of turn
         psidot = V/R; 
         thetadot = (q - (cos(theta)*sin(phi)*psidot))/cos(phi);             
         r = (-sin(phi)*thetadot)+ ((cos(theta)*cos(phi))*psidot);         
    end
    
    % Update quaternions
    euler = [phi;theta;psi].*(180/pi);
    quat = e2q(euler);
    q0 = quat(1);
    q1 = quat(2);
    q2 = quat(3);
    q3 = quat(4);    

    X0 = [u,v,w,p,q,r,q0,q1,q2,q3,xe,ye,ze]';

    if (Xbar_size==6)
        Xbar0 = [u,v,w,p,q,r]';
    elseif (Xbar_size==5)
        Xbar0 = [u,w,p,q,r]';    
    elseif (Xbar_size==3)
        Xbar0 = [u,w,q]'; 
    else
        disp("Incorrect input");
    end
end

        
        
          