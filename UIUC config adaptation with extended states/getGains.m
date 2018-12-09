function gains = getGains(params,eq)

    gains = struct;

    syms ps qs rs nxs nys nzs u1s u2s w1 w2 w3 w4 real
    wBs = [ps;qs;rs];
    ns = [nxs;nys;nzs];
    
    ss = [ps;qs;nxs;nys];
    us = [u1s;u2s];
    se = eq.s;
    ue = eq.u;
    wB = eq.wB;
    n = eq.n;
    w1_eq = sqrt(eq.f1/params.kF);
    w2_eq = sqrt(eq.f2/params.kF);
    w3_eq = sqrt(eq.f3/params.kF);
    w4_eq = 0;
    f = getReducedRates(ns,wBs,w1_eq,w2_eq,w3_eq,w4_eq,u1s,u2s,params);
    A = double(subs(jacobian(f,ss),[ss;us;rs;nzs],[se;ue;wB(3);n(3)]));
    B = double(subs(jacobian(f,us),[ss;us;rs;nzs],[se;ue;wB(3);n(3)]));

    [Ad, Bd] = make_discrete(A,B,params.dt);

    Q = diag([1 1 20 20]);
    R = diag([1 1]);
    gains.K_i = dlqr(Ad,Bd,Q,R);
    K_i = gains.K_i

    %now lets add the extended motor states
    %u = [f3-f1;f2] so I = eye(2) and O = zeros(2)
    Ae = [A B;
          zeros(2,4) -1/params.sigma_mot.*eye(2,2)];
    Be = [zeros(4,2);
          1/params.sigma_mot.*eye(2)];

    [Ad, Bd] = make_discrete(Ae,Be,params.dt);

    Q = diag([1 1 10 10 0 0]);
    R = diag([1 1]);
    gains.K_i_ext = dlqr(Ad,Bd,Q,R);
    K_i_ext = gains.K_i_ext
    
    %outter loop gains

    
    
    gains.K_px = 2*params.sigmax*params.omega_nx;
    gains.K_dx = params.omega_nx^2;
    
    gains.K_py = 2*params.sigmay*params.omega_ny;
    gains.K_dy = params.omega_ny^2;
    
    gains.K_pz = 2*params.sigmaz*params.omega_nz;
    gains.K_dz = params.omega_nz^2;
    
    
    K_px = gains.K_px
    K_dx = gains.K_dx
    
    K_py = gains.K_py
    K_dy = gains.K_dy
    
    K_pz = gains.K_pz
    K_dz = gains.K_dz
    
    
    A = [-K_px/K_dx 0 0;
         0 -K_py/K_dy 0;
         0 0 -K_pz/K_dz];
    B = [-1/K_dx 0 0;
         0 -1/K_dy 0;
         0 0 -1/K_dz];

    [Ad, Bd] = make_discrete(A,B,params.dt);

    Q = 2*eye(3);
    R = .1*eye(3);
    gains.K_o = dlqr(Ad,Bd,Q,R);
    
    K_o = gains.K_o
    
    
end