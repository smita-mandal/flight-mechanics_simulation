function [] = Main(simulation_case)
% simulation_case: An integer variable to simulate different scenarios with
% different control settings

    if (simulation_case==1)
        disp('Starting simulation for an elevator impulse of magnitude 3◦ and lasting 0.5 seconds');
    elseif (simulation_case==2)
        disp('Starting simulation for an aileron impulse of magnitude 3◦ and lasting 0.5 seconds');
    elseif (simulation_case==3)
        disp('Starting simulation for an rudder impulse of magnitude 3◦ and lasting 0.5 seconds');
    elseif (simulation_case==4)
        disp('Starting simulation for Red Bull Air Race course');
    else
        disp('Invalid Simulation Case');
        disp('Accepted values: Elevator Simulation(1),Aileron SImulation(2),Rudder Simulation(3),Red Bull Air Race(4)');
    end
    
    if (simulation_case==4)
        Xfinal = zeros(13,5);
        % Nominal CG case 180kn TAS at 20 m
        [FlightData,X0,U0] = Initialisation('CG1',180,simulation_case);        
        dt = 0.01;
        for i = 1:1:8
            switch (i)
                case 1
                    % Commencing aknife-edge manoeuvre after gate 1 sustaining the manoeuvre through gate 2
                    Gate2_dist = 150; % Distance from gate 1 to gate 2
                    time = 0;      
                    
                    % Simulating Gate1 to Gate2 with following conditions
                    X0(7) = 75;
                    X0(9) = 180;                    
                    trimmed = 0;
                    Xdot0 = zeros(13,1);

                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_mnvr_knifedge');
                    Xfinal = X;
                    Ufinal = U;
                    Xdotfinal  = Xdot;
                    PHI = euler;
                    t = time;
                    trimmed = 1;
                    while(abs(X(11))<= Gate2_dist)                         
                        time = time +dt;
                        X0 = X;                        
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_mnvr_knifedge');
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time                        
                    end                   
                    
                case 2
                    % Perform a full anti-clockwise 360◦ roll (coordinated with rudder to keep β = 0)
                    % The aircraft is flying at steady level flight instead
                    % of anticlockwise roll here
                    Gate3_dist1 = 400;                    
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = 0;
                    X0(9) = 180;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_level');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate3_dist1)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_level'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end                 
                    X0 = X;
                    % Replace quaternions with euler angles in the state vector
                    insert_index = 7;
                    part1 = X0(1:insert_index - 1);
                    part2 = X0(insert_index+4:end);                    
                    % State Vector with quaternions
                    X0 = [part1; euler;part2];
                    U0 = U;
                    Xdot0 = Xdot;

                    %-----------------------------------------------------------------------
                    % Covering 50 m with -180 degrees roll
                    %-----------------------------------------------------------------------
                    Gate3_dist2 = 500;                    
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = 0;
                    X0(9) = 90;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_level');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate3_dist2)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_level'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end               
                    
                    
                case 3
                    % Perform a constant-g turn from gate 3 to 4.
                    Gate4_dist = 750;
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = -45;
                    X0(9) = 90;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate4_dist)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end        
                    
                case 4
                    % Perform a constant-g turn from gate 4 to 5.
                    Gate5_dist = 1050;
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = -45;
                    X0(9) = 330;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate5_dist)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end                     
                    
                case 5
                    % Perform a constant-g turn from gate 5 to 6.
                    Gate6_dist = 1200;
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = -30;
                    X0(9) = 120;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate6_dist)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end                    
                case 6
                    % Steady level flight gate 6 to 7.
                    Gate7_dist = 1550;
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = 0;
                    X0(9) = 330;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate7_dist)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end 
                    
                case 7
                    % Steady inverted flight from Gate 8 to 1
                    Gate8_dist = 1750;
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = 180;
                    X0(9) = 330;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate8_dist)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end                                         
                case 8
                    Gate1_dist = 1956.2178;
                    time = time + dt;                    
                    % Simulating steady flight till 250 m                    
                    X0(7) = -45;
                    X0(9) = 180;
                    trimmed = 0; 
                    % Calculate state vector, control setting and attitude
                    % at initial time step
                    [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn');
                    % Save data
                    Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                    Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                    Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                    PHI = [PHI,euler];  % Save attitude for current simulation
                    t = [t,time];      % Save time
                    trimmed = 1;
                    while(abs(X(11))<= Gate1_dist)  
                        time = time +dt;
                        X0 = X;
                        U0 = U;
                        Xdot0 = Xdot;
                        % Calculate state vector, control setting and attitude
                        % with increment of time                        
                        [X,Xdot,U,euler] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'steady_turn'); 
                        % Save data
                        Xfinal  = [Xfinal,X];  % Save State Vector for current simulation          
                        Ufinal  = [Ufinal,U];  % Save Control Setting for current simulation
                        Xdotfinal  = [Xdotfinal,Xdot]; % Save State Rate Vector for current simulation
                        PHI = [PHI,euler];  % Save attitude for current simulation
                        t = [t,time];      % Save time
                    end 
                    
                otherwise
                    disp('end of simulation');
            
            end
            if (i==8)
                j = 1;
            else
                j = i+1;
            end
            fprintf("Time Taken to fly till Gate:%d is %f seconds\n",j,time);
            X0 = X;
            % Replace quaternions with euler angles in the state vector
            insert_index = 7;
            part1 = X0(1:insert_index - 1);
            part2 = X0(insert_index+4:end);                    
            % State Vector with quaternions
            X0 = [part1; euler;part2];
            U0 = U;
            Xdot0 = Xdot;            
        end        
    else               
        time = 0;
        dt = 0.01; % Time Step
        Xdot0 = zeros(13,1);
        t = 0:0.01:1; % TIme flor the simulations run
        trimmed =1;

        % Nominal CG case 100kn TAS at 1000ft
        [FlightData,X0,U0] = Initialisation('CG1',100,simulation_case);        
        [Xfinal,Xdotfinal,Ufinal,PHI] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',100,simulation_case,trimmed,'default');
        
        % Nominal CG case 180kn TAS at 1000ft
        [FlightData,X0,U0] = Initialisation('CG1',180,simulation_case); 
        [Xfinal,Xdotfinal,Ufinal,PHI] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG1',180,simulation_case,trimmed,'default');

        % 2nd CG case 100kn TAS at 1000ft
        [FlightData,X0,U0] = Initialisation('CG2',100,simulation_case);
        [Xfinal,Xdotfinal,Ufinal,PHI] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG2',100,simulation_case,trimmed,'default');

        % 2nd CG case 180kn TAS at 1000ft
        [FlightData,X0,U0] = Initialisation('CG2',180,simulation_case);
        [Xfinal,Xdotfinal,Ufinal,PHI] = Flight_Simulation(FlightData,X0,Xdot0,U0,time,dt,'CG2',180,simulation_case,trimmed,'default');       
        
        
    end
    % Plot data 
    PlotResults(Xfinal,Xdotfinal,Ufinal,PHI,t);     
end

    
        




