function [Xdot0,Xbardot]= Trim_State_rate(FlightData,X0,Xbar_size) 
    
    % Unpack state vector variables
    u = X0(1);
    v = X0(2);
    w = X0(3);
    p = X0(4);
    q = X0(5);
    r = X0(6);
    q0 = X0(7);
    q1 = X0(8);
    q2 = X0(9);
    q3 = X0(10);

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

    %Find Earth to body DCM
    L_be = DCM(X0);   

    %Find change in velocity with time     
    udot = (r*v)-(q*w);
    vdot = (-r*u)+(p*w);
    wdot = (q*u)-(p*v);
    
    % Calculate change in body rate with time
    pdot = (C3*p*q)+(C4*q*r);
    qdot = (C7*p*r)-(C6*((p^2)-(r^2)));
    rdot = (C9*p*q)-(C3*q*r);

    %Find quaternion rates
    q0dot = (-1/2)*((q1*p)+(q2*q)+(q3*r));
    q1dot = (1/2)*((q0*p)-(q3*q)+(q2*r));
    q2dot = (1/2)*((q3*p)+(q0*q)-(q1*r));
    q3dot = (-1/2)*((q2*p)-(q1*q)-(q0*r));    

    %Find earth position components xdot ydot zdot
    earth = inv(L_be)*[u,v,w]';
    xedot = earth(1);
    yedot = earth(2);
    zedot = earth(3);

    % Form State Rate vector
    Xdot0 = [udot,vdot,wdot,pdot,qdot,rdot,q0dot,q1dot,q2dot,q3dot,xedot,yedot,zedot]';    

    if (Xbar_size==6)
        Xbardot(1) = Xdot0(1);
        Xbardot(2) = Xdot0(2);
        Xbardot(3) = Xdot0(3);
        Xbardot(4) = Xdot0(4);
        Xbardot(5) = Xdot0(5); 
        Xbardot(6) = Xdot0(6);
    elseif (Xbar_size==5)
        Xbardot(1) = Xdot0(1);
        Xbardot(2) = Xdot0(3);
        Xbardot(3) = Xdot0(4);
        Xbardot(4) = Xdot0(5);
        Xbardot(5) = Xdot0(6);            
     else            
        Xbardot(1) = Xdot0(1);
        Xbardot(2) = Xdot0(3);
        Xbardot(3) = Xdot0(5);        
     end 
     Xbardot = Xbardot';
end

        
        
          