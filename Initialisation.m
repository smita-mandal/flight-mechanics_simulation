function [FlightData,X0,U0] = Initialisation(CG_case,IC,simulation_case)
% CG_case: A string specifying the case ("CG1" or "CG2")
% IC: An integer variable specifying the case of different State Vector and Control Settings
% simulation_case: An integer variable to simulate different scenarios with
% different control settings

    if (simulation_case ~= 4) % Simulation for Elevator/Aileron/Rudder Impulse
        if strcmp(CG_case, 'CG1')
            [FlightData] = aero3560_LoadFlightDataPC9_nominalCG1();
            if (IC==100) % 100kn TAS at 1000ft
                load("ICs_PC9_nominalCG1_100Kn_1000ft","X0");            
                load("ICs_PC9_nominalCG1_100Kn_1000ft","U0");            
            elseif (IC==180) % 180kn TAS at 1000ft
                load("ICs_PC9_nominalCG1_180Kn_1000ft","X0");
                load("ICs_PC9_nominalCG1_180Kn_1000ft","U0");
            end
        elseif strcmp(CG_case, 'CG2')
            [FlightData] = aero3560_LoadFlightDataPC9_CG2();
            if (IC==100) % 100kn TAS at 1000ft
                load("ICs_PC9_CG2_100Kn_1000ft","X0");
                load("ICs_PC9_CG2_100Kn_1000ft","U0");
            elseif (IC==180) % 180kn TAS at 1000ft
                load("ICs_PC9_CG2_180Kn_1000ft","X0");
                load("ICs_PC9_CG2_180Kn_1000ft","U0");
            end
        else
            disp('Incorrect entry for CG case: Enter CG1 or CG2');
        end 
    else % Red Bull Air Race Simulation
        % Initialise Aerodynamic and Geometric values per the aircraft
        [FlightData] = aero3560_LoadFlightDataPC9_nominalCG1();
        % Load initial state vector and control setting data                        
        load("ICs_PC9_nominalCG1_180Kn_20m_RedBull_Race_State","X0");
        load("ICs_PC9_nominalCG1_180Kn_20m_RedBull_Race_Control","U0");       
                
    end

end
