✈️ Pilatus PC-9 Flight Mechanics Simulation & Handling Qualities Analysis

📌 **Overview:**

This repository presents the development of a MATLAB-based flight simulator for the PC-9 aircraft, focusing on flight dynamics modelling and handling qualities assessment. The project simulates aircraft response to control inputs and evaluates stability characteristics using linearised equations of motion and eigenvalue analysis.

🎯 **Objectives:**
* Develop a flight dynamics simulation framework for the PC-9 aircraft
* Analyse aircraft response to control surface inputs (elevator, aileron, rudder)
* Compute trim conditions for steady flight
* Evaluate handling qualities and stability characteristics
* Study aircraft behaviour across different CG positions and flight conditions

⚙️ **Methodology:**

🔹 Flight Simulation:
The simulator is built using modular MATLAB functions to-
* Initialise aircraft parameters and configurations
* Compute aerodynamic properties and forces
* Determine trim conditions using Newton-Raphson method
* Simulate aircraft motion using 6-DOF equations of motion
* Integrate dynamics using Runge-Kutta (RK4) method
  
🔹 Control Inputs & Scenarios:
The aircraft response is analysed for-
* Elevator impulse
* Aileron impulse
* Rudder impulse
* Red Bull Air Racecourse manoeuvre simulation

These scenarios evaluate different modes of motion such as short-period, roll, and coupled dynamics.

📊 **Handling Qualities Analysis:**
* Linearised equations of motion are used to compute A matrices
* Eigenvalue analysis is performed to assess-
  **Stability of modes (short period, phugoid, roll, Dutch roll, spiral)
  **Effects of speed and CG location
* Results are compared against military handling quality standards
  
🛠️ **Code Structure:**

The simulation follows a modular architecture-


<img width="700" height="690" alt="image" src="https://github.com/user-attachments/assets/eafb8d1a-b19c-446c-b67b-3d64073ae010" />


📌 The workflow includes initialisation → trim → simulation → integration → plotting, as shown in the simulation flow diagram .

📈 **Key Features:**
* Full 6-DOF flight dynamics simulation
* Trim computation for steady and manoeuvre conditions
* Control input modelling (impulse-based)
* Handling qualities evaluation using eigenvalues
* Simulation of complex manoeuvres (air racecourse)
  
📊 **Results & Insights:**
* Elevator input shows short-period longitudinal dynamics
* Aileron input highlights roll mode behaviour with damping
* Rudder input exhibits weak coupling in yaw dynamics
* Stability characteristics vary with speed and CG location
  
📚 **References:**
* Nelson, Flight Stability and Automatic Control, 2nd Edn, 1998, McGraw-Hill McCormick, Aerodynamics, Aeronautics and Flight Mechanics, 2nd Edn, 1995, Wiley
Etkin, Dynamics of Atmospheric Flight, 1972
* Roskam, Airplane Flight Dynamics and Automatic Flight Controls AIAA ANSI R-004-1992, Recommended Practice– Atmospheric and Space Flight Ve
hicle Coordinate Systems
* MATLAB Documentations

🤝 **Acknowledgment:**

This project was completed as part of AERO3560 – Flight Mechanics 1 at the University of Sydney, involving collaborative development of the simulation and analysis framework.

🚀 **Key Takeaway:**

This project demonstrates the integration of flight dynamics modelling, control response simulation, and stability analysis, providing a comprehensive framework for evaluating aircraft performance and handling qualities.

📄 **Documentation:**

* 📘 [PC-9 Flight Simulation and Stability Analysis Report](docs/pc9-flight-simulation-and-stability-analysis-report.pdf)
-------------------------------------------------------------------------------------------------------------------------------------------------------------------
📁 **Repository Structure:**

/src           → MATLAB source code  
/data          → Supporting MATLAB data files  
/results       → Plots and simulation outputs  
/docs          → Supporting documentation 


▶️ **Instructions to Run:**
1. Open MATLAB and navigate to the project directory.
2. Run the main function by specifying the desired simulation case:
   </> MATLAB
    Main(simulation_case)
3. Select a simulation scenario using the following options:
   * Main(1) → Elevator impulse (3° for 0.5 seconds)
   * Main(2) → Aileron impulse (3° for 0.5 seconds)
   * Main(3) → Rudder impulse (3° for 0.5 seconds)
   * Main(4) → Red Bull Air Race course simulation
4. The simulation will automatically:
   * Initialise aircraft parameters
   * Compute trim conditions
   * Run flight dynamics simulation
   * Generate plots for state variables and control inputs

💡 **Example:**
   Main(1)   % Run elevator impulse simulation

⚠️ **Note:**
   Ensure all required MATLAB files are in the same directory or added to the MATLAB path before running the simulation.
  
  -----------------------------------------------------------------------------------------------------------------------------------------------------------------
