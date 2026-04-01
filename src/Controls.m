function [Un] = Controls(CG_case,IC,simulation_case,iter)

    % Load control setting data for each simulation case for a time span of
    % 1 second   
    
    if (simulation_case == 1)
        if strcmp(CG_case, 'CG1')
            if (IC==100) % 100kn TAS at 1000ft
                load("U_filter_elevator_Impulse_3deg_0.5sec_Sim2sec_CG1_100","U_filter");           
            elseif (IC==180) % 180kn TAS at 1000ft
                load("U_filter_elevator_Impulse_3deg_0.5sec_Sim2sec_CG1_180","U_filter");
            end
        elseif strcmp(CG_case, 'CG2')
            if (IC==100) % 100kn TAS at 1000ft
                load("U_filter_elevator_Impulse_3deg_0.5sec_Sim2sec_CG2_100","U_filter");           
            elseif (IC==180) % 180kn TAS at 1000ft
                load("U_filter_elevator_Impulse_3deg_0.5sec_Sim2sec_CG2_180","U_filter");
            end
        end
    elseif  (simulation_case == 2)
        if strcmp(CG_case, 'CG1')
            if (IC==100) % 100kn TAS at 1000ft
                load("U_filter_aileron_Impulse_3deg_0.5sec_Sim2sec_CG1_100","U_filter");           
            elseif (IC==180) % 180kn TAS at 1000ft
                load("U_filter_aileron_Impulse_3deg_0.5sec_Sim2sec_CG1_180","U_filter");
            end
        elseif strcmp(CG_case, 'CG2')
            if (IC==100) % 100kn TAS at 1000ft
                load("U_filter_aileron_Impulse_3deg_0.5sec_Sim2sec_CG2_100","U_filter");           
            elseif (IC==180) % 180kn TAS at 1000ft
                load("U_filter_aileron_Impulse_3deg_0.5sec_Sim2sec_CG2_180","U_filter");
            end
        end
     elseif  (simulation_case == 3)
        if strcmp(CG_case, 'CG1')
            if (IC==100) % 100kn TAS at 1000ft
                load("U_filter_rudder_Impulse_3deg_0.5sec_Sim2sec_CG1_100","U_filter");           
            elseif (IC==180) % 180kn TAS at 1000ft
                load("U_filter_rudder_Impulse_3deg_0.5sec_Sim2sec_CG1_180","U_filter");
            end
        elseif strcmp(CG_case, 'CG2')
            if (IC==100) % 100kn TAS at 1000ft
                load("U_filter_rudder_Impulse_3deg_0.5sec_Sim2sec_CG2_100","U_filter");           
            elseif (IC==180) % 180kn TAS at 1000ft
                load("U_filter_rudder_Impulse_3deg_0.5sec_Sim2sec_CG2_180","U_filter");
            end
        end   
    else
        disp('Incorrect Simulation Case');
    end
    U_filter = U_filter.*(pi/180);
    Un = U_filter(:,iter);     

end
      
