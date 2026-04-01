function Xdot = StateRates(X,Xdot,U,FlightData,time)
    % Update the state rates vector based on estimates from the initial state
    m = FlightData.Inertial.m;
    g = FlightData.Inertial.g;
    u = X(1);
    v = X(2);
    w = X(3);
    p = X(4);
    q = X(5);
    r = X(6);
    q0 = X(7);
    q1 = X(8);
    q2 = X(9);
    q3 = X(10);
    %Find gravity in body axis
    Gbe = Gravity(X,FlightData.Inertial.g);
    %Find Forces and Moments
    [Forces, Moments] = BodyForces(X,Xdot,U,FlightData,time);
    
    %Find Earth to body DCM
    L_be = DCM(X);
    
    %Find Inertial Coefficients
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
    
    %Find body udot vdot wdot     
    udot = (r*v)-(q*w)+Gbe(1)+(Forces(1)/m);
    vdot = (-r*u)+(p*w)+Gbe(2)+(Forces(2)/m);
    wdot = (q*u)-(p*v)+Gbe(3)+(Forces(3)/m);
    
    %Find Body rotation rates pdot qdot rdot
    L = Moments(1);
    M = Moments(2);
    N = Moments(3);
    pdot = (C3*p*q)+(C4*q*r)+(C1*L)+(C2*N);
    qdot = (C7*p*r)-(C6*((p^2)-(r^2)))+(C5*M);
    rdot = (C9*p*q)-(C3*q*r)+(C2*L)+(C8*N);
    
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
    
    %Combine Xdot vector
    Xdot = [udot,vdot,wdot,pdot,qdot,rdot,q0dot,q1dot,q2dot,q3dot,xedot,yedot,zedot]';
    
end
