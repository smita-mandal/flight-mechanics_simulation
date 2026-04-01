function[] = PlotResults(X,Xdot,U,euler,time)    
    
    %Plot the saved Data 
    
    for i= 1:1:length(time)
        %For State values
        u(i) = X(1,i);         
        v(i) = X(2,i);
        w(i) = X(3,i);
        p(i) = X(4,i).*(180/pi);
        q(i) = X(5,i).*(180/pi);
        r(i) = X(6,i).*(180/pi);                
        phi(i) = euler(1,i);
        theta(i) = euler(2,i);
        psi(i) = euler(3,i);
        x(i) = X(11,i);
        y(i) = X(12,i);
        z(i) = X(13,i); 
        dT(i) = U(1,i);
        de(i) = U(2,i).*(180/pi);
        da(i) = U(3,i).*(180/pi);
        dr(i) = U(4,i).*(180/pi);       
    
    end
    %States Variable plots    
    figure(1);
    subplot(3,1,1);    
    plot(time,u,'o-',LineWidth=1);  
    xlabel('Time(s)');
    ylabel('Velocity in X-Direction (m/s)');
    title('Velocity of Aircraft in X-direction vs Time');    
    
    subplot(3,1,2);
    plot(time,v,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Velocity in Y-Direction (m/s)');
    title('Velocity of Aircraft in Y-direction vs Time');
    
    subplot(3,1,3);
    plot(time,w,'o-',LineWidth=1);
    xlabel('Time(s)');
    ylabel('Velocity in Z-Direction (m/s)');
    title('Velocity of Aircraft in Z-direction vs Time');
    
    figure(2);
    subplot(3,1,1);
    plot(time,p,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Roll rate (degrees/s)');
    title('Body rate of Aircraft about X-axis vs Time');
    
    subplot(3,1,2);
    plot(time,q,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Pitch rate (degrees/s)');
    title('Body rate of Aircraft about Y-axis vs Time');
    
    subplot(3,1,3);
    plot(time,r,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Yaw rate (degrees/s)');
    title('Body rate of Aircraft about Z-axis vs Time');
    
    %Attitude Plot
    figure(3);
    subplot(3,1,1);
    plot(time,phi,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Roll Angle (degrees)');
    title('Euler Angle of Aircraft about X-axis vs Time');
    
    subplot(3,1,2);
    plot(time,theta,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Pitch Angle (degrees)');
    title('Euler Angle of Aircraft about Y-axis vs Time');
    
    subplot(3,1,3);
    plot(time,psi,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Yaw Angle (degrees)');
    title('Euler Angle of Aircraft about Z-axis vs Time');
    
    figure(4);
    subplot(3,1,1);
    plot(time,x,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Position in X-axis (m)');
    title('Position of Aircraft along X-axis vs Time');
    
    subplot(3,1,2);
    plot(time,y,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Position in Y-axis  (m)');
    title('Position of Aircraft along Y-axis vs Time');
    
    subplot(3,1,3);
    plot(time,-(z),'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Position in Z-axis (m)');
    title('Position of Aircraft along Z-axis vs Time');

    figure(5);
    subplot(2,1,1);
    plot(time,dT,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Thrust Setting ');
    title('Thrust Setting of Aircraft vs Time');

    subplot(2,1,2);
    plot(time,de,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Elevator Deflection (degrees)');
    title('Elevator Deflection of Aircraft vs Time');

    figure(6);
    subplot(2,1,1);
    plot(time,da,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Aileron Deflection (degrees)');
    title('Aileron Deflection of Aircraft vs Time');

    subplot(2,1,2);
    plot(time,dr,'o-',LineWidth=1);    
    xlabel('Time(s)');
    ylabel('Rudder Deflection (degrees)');
    title('Rudder Deflection of Aircraft vs Time'); 

end
