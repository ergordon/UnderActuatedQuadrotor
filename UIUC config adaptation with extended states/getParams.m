function params = getParams(eq)

    params = struct;
    
    %initial conditions
    params.o0 = [0; 0; -1];
    params.theta0 = [pi/2; 0; 0];
    params.v0 = [0; 0; 0];
    params.w0 = [0;0;0];
    params.motorRates0 = [0;0;0;0];
    params.x0 = [params.o0; params.theta0; params.v0; params.w0;params.motorRates0];
    
    
    %time
    params.t0 = 0;  %simulation start time
    params.t1 = 5; %simulation stop time
    params.dt = 1/1000; %frequency of inner loop
    
    
    
    params.frequencyRatio = 5; %ratio of outter to inner loop frequencies
    params.sigma_mot = .015; %time constant for motors
    params.el = .17; %quadrotor spar length
    params.kF = 7.46e-6; %motor->force coefficient
    params.kT = 1.23e-7/params.kF; %motor->torque coefficient
    params.JT = diag([4093e-6 3944e-6 7593e-6]); %total moment of inertia
    params.JP = diag([0 0 1.5e-11]); %rotor moment of inertia
    params.JB = params.JT - 4*params.JP; %moment of inertia less motors
    params.sigmamax = 1000; %max rotor spin rate
    params.g = 9.81; %gravity
    params.m = 0.715; %mass
    params.gamma = 2.75e-3; %yaw drag coefficient
    
    
    %PID gains used for the outter loop (actually only PD)
    params.sigmax = .5;%damping ratio
    params.omega_nx = .5;%natural frequency
    
    params.sigmay = .5;%damping ratio
    params.omega_ny = .5;%natural frequency
    
    params.sigmaz = .7;%damping ratio
    params.omega_nz = 1;%natural frequency
    

end