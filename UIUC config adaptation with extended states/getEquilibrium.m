function eq = getEquilibrium()

    eq = struct;
    
    load('equilib.mat');
    eq.f1 = f1_eq;
    eq.f2 = f2_eq;
    eq.f3 = f3_eq;
    eq.f4 = f4_eq;
    eq.n = [nx_eq;ny_eq;nz_eq];
    eq.wB = [p_eq;q_eq;r_eq];
    
    %equilibrium reduced attitude and input
    eq.s = [p_eq;q_eq;nx_eq;ny_eq]; %[p;q;nx;ny]
    eq.u = [f2_eq-f1_eq;f3_eq]; %equilibrium (force) input for inner loop reduced state
    eq.a = [0;0;0]; %equilibrium input (acceleration) for outter loop
    eq.s_ext = [p_eq;q_eq;nx_eq;ny_eq;f2_eq-f1_eq;f3_eq]; %[p;q;nx;ny;f2-f1;f1]
    
end