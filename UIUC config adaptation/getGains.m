function gains = getGains(params,eq)

    gains = struct;

%     a_eq = (params.JT(1,1)-params.JT(3,3))/params.JB(1,1)*eq.wB(3) - params.JP(3,3)/params.JB(1,1)*(eq.w1+eq.w2+eq.w3+eq.w4);
%     A = [0 a_eq 0 0;
%          -a_eq 0 0 0;
%          0 -eq.n(3) 0 eq.wB(3);
%          eq.n(3) 0 -eq.wB(3) 0];
% 
%     B = params.el/params.JB(1,1)*[0 1;
%                                   1 0;
%                                   0 0;
%                                   0 0];
    syms ps qs rs nxs nys nzs u1s u2s w1 w2 w3 w4 real
    wBs = [ps;qs;rs];
    ns = [nxs;nys;nzs];
    
    ss = [ps;qs;nxs;nys];
    us = [u1s;u2s];
    se = eq.s;
    ue = eq.u;
    wB = eq.wB;
    n = eq.n;
    f = getReducedRates(ns,wBs,eq.w1,eq.w2,eq.w3,eq.w4,u1s,u2s,params);
    A = double(subs(jacobian(f,ss),[ss;us;rs;nzs],[se;ue;wB(3);n(3)]));
    B = double(subs(jacobian(f,us),[ss;us;rs;nzs],[se;ue;wB(3);n(3)]));

    [Ad, Bd] = make_discrete(A,B,params.dt);

    Q = diag([1 1 100 100]);
    R = diag([1 1]);
    gains.K_i = dlqr(Ad,Bd,Q,R);
    K_i = gains.K_i;

    %now lets add the extended motor states
    %u = [f3-f1;f2] so I = eye(2) and O = zeros(2)
    Ae = [A B;
          zeros(2,4) -params.sigma_mot^(-1)*eye(2,2)];
    Be = [zeros(4,2);
          params.sigma_mot^(-1)*eye(2)];

    [Ad, Bd] = make_discrete(Ae,Be,params.dt);

    Q = diag([1 1 100 100 0 0]);
    R = diag([1 1]);
    gains.K_i_ext = dlqr(Ad,Bd,Q,R);
    K_i_ext = gains.K_i_ext;
    
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
    
    K_p = gains.K_p;
    K_d = gains.K_d;
end