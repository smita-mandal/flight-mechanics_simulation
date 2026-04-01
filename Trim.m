function [Xt,Xtdot,Ut]= Trim(FlightData,Xinit,U0,V,trim_case)         
    
    % Trimming for various cases
    if (strcmp(trim_case, 'steady_mnvr_knifedge'))       
        Xbar_s = ["alpha","beta","dT","de","da","dr"];        
        Xbar_size = 6;                      
        J = zeros(6,6);
    elseif (strcmp(trim_case, 'steady_turn'))        
        Xbar_s = ["alpha","dT","de","da","dr"];        
        Xbar_size = 5;                      
        J = zeros(5,5);    
    elseif strcmp(trim_case, 'steady_level')
        Xbar_s = ["alpha","dT","de"];
        Xbar_size = 3;
        J = zeros(3,3);       
    else
        disp('Incorrect Trimming Case');
    end 

    [X0,Xbar0,alpha0,beta0,gamma,rho,Qbar,CL] = Trim_inputs(FlightData,Xbar_size,Xinit,V,trim_case);
    
    % Initial values
    Xbar = Xbar0;
    alpha_old = alpha0;
    beta_old = beta0;    
    
    % Iteration constraints
    dx = 0.0001;   % Perturbation value
    kmax = 50;       % maximum allowed iteration    
    error = 1;        % define error     
    tolerance = 0.001; % define convergence tolerance
    k = 1;
    Xk0 = X0;
        
    while ((k<kmax) && (error > tolerance))
                
        if (k~=1)
        [alpha_old,beta_old,U0,V,Xk0,Xbar0]= Trim_control(FlightData,trim_case,rho,Qbar,CL,Xk0,Xbar_size,Xbar);        
        end
        dT_old = U0(1);
        de_old = U0(2);
        da_old = U0(3);
        dr_old = U0(4);        
        
        % Calculate State Rate Vector        
        [Xkdot0,Xbardot0]= Trim_State_rate(FlightData,Xk0,Xbar_size);

        for i = 1:1:Xbar_size
          if (strcmp(Xbar_s(i),"alpha"))
              j = 1;      
          elseif (strcmp(Xbar_s(i),"beta"))
              j = 2;      
          elseif (strcmp(Xbar_s(i),"dT"))
              j = 3;
          elseif (strcmp(Xbar_s(i),"de"))
              j = 4;
          elseif (strcmp(Xbar_s(i),"da"))
              j = 5;
          elseif (strcmp(Xbar_s(i),"dr"))
              j = 6;
          else
              j = 0;
          end

          % Forward and Backward perturbation        
          switch(j)
              case 1
                  alpha_newf = alpha_old+dx;
                  alpha_newb = alpha_old-dx;         
                  
                  % Form State Vector
                  Xbardot0f = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0,V,alpha_newf,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
                  Xbardot0b = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0,V,alpha_newb,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));      
              case 2
                  beta_newf = beta_old+dx;
                  beta_newb = beta_old-dx;         
                  
                  % Form State Vector
                  Xbardot0f = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0,V,alpha_old,beta_newf,gamma,rho,Qbar,CL,Xbar_s(i));
                  Xbardot0b = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0,V,alpha_old,beta_newb,gamma,rho,Qbar,CL,Xbar_s(i));
              case 3
                  dT_newf = dT_old + dx;
                  dT_newb = dT_old - dx;
                  U0f = U0;
                  U0b = U0;
      
                  U0f(1) = dT_newf;
                  U0b(1) = dT_newb; 
                  
                  Xbardot0f = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0f,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
                  Xbardot0b = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0b,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
              case 4
                  de_newf = de_old + dx;
                  de_newb = de_old - dx;
                  U0f = U0;
                  U0b = U0;
      
                  U0f(2) = de_newf;
                  U0b(2) = de_newb;  
                  
                  Xbardot0f = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0f,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
                  Xbardot0b = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0b,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
              case 5
                  da_newf = da_old + dx;
                  da_newb = da_old - dx;
                  U0f = U0;
                  U0b = U0;
      
                  U0f(3) = da_newf;
                  U0b(3) = da_newb;                    
              
                  Xbardot0f = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0f,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
                  Xbardot0b = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0b,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
              case 6
                  dr_newf = dr_old + dx;
                  dr_newb = dr_old - dx;
                  U0f = U0;
                  U0b = U0;
      
                  U0f(4) = dr_newf;
                  U0b(4) = dr_newb; 
                  
                  Xbardot0f = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0f,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
                  Xbardot0b = Trim_inputs_perturbation(FlightData,Xbar_size,Xk0,Xkdot0,U0b,V,alpha_old,beta_old,gamma,rho,Qbar,CL,Xbar_s(i));
              otherwise
                    disp('incorrect input');
            
          end          
           
          if (Xbar_size==6)
            J(1,i) = (Xbardot0f(1) - Xbardot0b(1))/(2*dx);
            J(2,i) = (Xbardot0f(2) - Xbardot0b(2))/(2*dx);
            J(3,i) = (Xbardot0f(3) - Xbardot0b(3))/(2*dx);
            J(4,i) = (Xbardot0f(4) - Xbardot0b(4))/(2*dx);
            J(5,i) = (Xbardot0f(5) - Xbardot0b(5))/(2*dx); 
            J(6,i) = (Xbardot0f(6) - Xbardot0b(6))/(2*dx);  
          elseif (Xbar_size==5)
            J(1,i) = (Xbardot0f(1) - Xbardot0b(1))/(2*dx);
            J(2,i) = (Xbardot0f(2) - Xbardot0b(2))/(2*dx);
            J(3,i) = (Xbardot0f(3) - Xbardot0b(3))/(2*dx);
            J(4,i) = (Xbardot0f(4) - Xbardot0b(4))/(2*dx);
            J(5,i) = (Xbardot0f(5) - Xbardot0b(5))/(2*dx);       
          
          else
            J(1,i) = (Xbardot0f(1) - Xbardot0b(1))/(2*dx);
            J(2,i) = (Xbardot0f(2) - Xbardot0b(2))/(2*dx);
            J(3,i) = (Xbardot0f(3) - Xbardot0b(3))/(2*dx);              
          end        
          
        end
        
        Xbar = Xbar0 - (pinv(J)*Xbardot0);               
        
        % Calculate error        
        error = sum(abs(Xbar - Xbar0));
        
        J =zeros(Xbar_size,Xbar_size);
        k = k+1;
    end
    
    % Update Control Setting
    [alpha,beta,Ut,V,Xt,Xbar0]= Trim_control(FlightData,trim_case,rho,Qbar,CL,Xk0,Xbar_size,Xbar);
    
    % Form State Vector 
    [Xtdot,Xbardot0]= Trim_State_rate(FlightData,Xt,Xbar_size);  
         
end





                
            
            
            
            
          
            
            
            
  
            
            
  
         
    
            
            
          
            
          
            
          
          
         
        
        
        
        
    