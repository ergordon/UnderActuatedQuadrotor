function eq = getEquilibrium()

    eq = struct;
    
    load('equilib.mat');
    params.kF = 6.41*10^(-6);
    eq.f1 = f1_eq;
    eq.f2 = f2_eq;
    eq.f3 = f3_eq;
    eq.f4 = f4_eq;
    eq.n = [nx_eq;ny_eq;nz_eq];
    eq.wB = [p_eq;q_eq;r_eq];
    eq.w1 = w1_eq;
    eq.w2 = w2_eq;
    eq.w3 = w3_eq;
    eq.w4 = w4_eq;
  
    %equilibrium reduced attitude and input
    eq.s = [p_eq;q_eq;nx_eq;ny_eq]; %[p;q;nx;ny]
    eq.u = [f3_eq-f1_eq;f2_eq]; %equilibrium (force) input for inner loop reduced state
    eq.a = [0;0;0]; %equilibrium input (acceleration) for outter loop
    
    
end