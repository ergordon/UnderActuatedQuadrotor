function gains = getGains(params,eq)

    gains = struct;

    a_eq = (params.JT(1,1)-params.JT(3,3))/params.JB(1,1)*eq.wB(3) - params.JP(3,3)/params.JB(1,1)*(eq.w1+eq.w2+eq.w3+eq.w4);
    A = [0 a_eq 0 0;
         -a_eq 0 0 0;
         0 -eq.n(3) 0 eq.wB(3);
         eq.n(3) 0 -eq.wB(3) 0];

    B = params.el/params.JB(1,1)*[0 1;
                                  1 0;
                                  0 0;
                                  0 0];
    [Ad, Bd] = make_discrete(A,B,params.dt);

    Q = diag([1 1 20 20]);
    R = diag([1 1]);
    gains.K_i = dlqr(Ad,Bd,Q,R);

    %now lets add the extended motor states
    %u = [f3-f1;f2] so I = eye(2) and O = zeros(2)
    Ae = [A B;
          zeros(2,4) -params.sigma_mot^(-1)*eye(2,2)];
    Be = [zeros(4,2);
          params.sigma_mot^(-1)*eye(2)];

    [Ad, Bd] = make_discrete(Ae,Be,params.dt);

    Q = diag([1 1 20 20 0 0]);
    R = diag([1 1]);
    gains.K_i_ext = dlqr(Ad,Bd,Q,R);

    
    %outter loop gains

%     A = -params.omega_n/(2*params.sigma)*ones(3,3);
%     B = -1/(2*params.sigma*params.omega_n)*ones(3,3);
% 
%     [Ad, Bd] = make_discrete(A,B,params.dt);
% 
%     Q = 1*eye(3);
%     R = 1*eye(3);
%     gains.K_o = dlqr(Ad,Bd,Q,R);
    
    gains.K_p = 2*params.sigma*params.omega_n;
    gains.K_d = params.omega_n^2;
    
end