function [X,Xdot,U,PHI] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,CG_case,IC,simulation_case,trimmed,trim_case)   
% Flight Data: Flight Aerodynamic and Geometric data
% X0: Initial State Vector
% U0: Initial Control Setting 
% Xdot0: State Rate Vector
% time: Current Time
% dt: Time step
% CG_case: A string specifying the case ("CG1" or "CG2")
% IC: An integer variable specifying the Velocity value
% simulation_case: An integer variable to simulate different scenarios with
% trim_case: a string specifying the case for trimming the controls

    if (simulation_case==4) 

        V = IC;   % Initial Velocity of the aircraft
        if (trimmed==0)
            [Xt,Xdott,Ut] = Trim(FlightData,X0,U0,V,trim_case);
            X0 = Xt;
            Xdot0 = Xdott;
            U0 = Ut;
        end
                
        %--------------------------------------------------------------
        % Runga Kutta 4th Order Integration to obtain estimated state
        % vector across time
        [Xn,Xdotn] = Integrate(FlightData,X0,Xdot0,U0,time,dt);                
        % Normalize quaternions to keep the magnitude as 1
        mu = sqrt((Xn(7)^2) + (Xn(8)^2) + (Xn(9)^2) + (Xn(10)^2));
        Xn(7) = (1/mu) * Xn(7) ;
        Xn(8) = (1/mu) * Xn(8) ;
        Xn(9) = (1/mu) * Xn(9) ;
        Xn(10) = (1/mu) * Xn(10);  
        
        % Convert Quaternions to Euler Angles
        quatn = [Xn(7);Xn(8);Xn(9);Xn(10)];
        eulern = q2e(quatn);        

        X = Xn;
        U = U0;
        Xdot = Xdotn;        
        PHI = eulern;                 
    else
        %--------------------------------------------------------------
        % Replace Euler angles with quaternions
        %--------------------------------------------------------------
        % Euler angles to Quaternions conversion
        euler0 = [X0(7);X0(8);X0(9)];
        [quat0] = e2q(euler0);   
        % Replace Euler angles with quaternions in the state vector
        insert_index = 7;
        part1 = X0(1:insert_index - 1);
        part2 = X0(insert_index+3:end);
        
        % State Vector with quaternions
        X0 = [part1; quat0;part2];            
        %--------------------------------------------------------------
        % Save state vector at time step =0 
        X  = X0; 
        % Save control setting at time step =0 
        U  = U0; 
        % Save attitude of aircraft at time step =0 
        PHI  = euler0;
        %--------------------------------------------------------------
        iter = 1;        
        %--------------------------------------------------------------
        
        for time = 0:0.01:1
            % Update control settings as per requested simulation scenario
            Un = Controls(CG_case,IC,simulation_case,iter);
            % Runga Kutta 4th Order Integration to obtain estimated state
            % vector across time
            [Xn,Xdotn] = Integrate(FlightData,X0,Xdot0,Un,time,dt);  
            
            % Normalize quaternions to keep the magnitude as 1
            mu = sqrt((Xn(7)^2) + (Xn(8)^2) + (Xn(9)^2) + (Xn(10)^2));
            Xn(7) = (1/mu) * Xn(7) ;
            Xn(8) = (1/mu) * Xn(8) ;
            Xn(9) = (1/mu) * Xn(9) ;
            Xn(10) = (1/mu) * Xn(10);  
            
            % Convert Quaternions to Euler Angles
            quatn = [Xn(7);Xn(8);Xn(9);Xn(10)];
            [eulern] = q2e(quatn);            

            % Save data
            X  = [X, Xn];  % Save State Vector for current time step            
            U  = [U, Un];  % Save Control Setting for current time step            
            % Save State Rate vector for current time step 
            if time ==0
               Xdot  = Xdot0;                
            else
                
               Xdot  = [Xdot,Xdotn];
            end
            PHI = [PHI,eulern];  % Save attitude for current time step                       

            X0 = Xn;
            Xdot0 = Xdotn;
            iter = iter + 1; 
        end
        
    end    
    
 end








