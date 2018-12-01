function u=Outer(t,x,dt)
%%% NOMINAL CONTROL - This is the baseline control which we used when we
% linearized the system

persistent Kout

dt = 0.01;
m = 0.715; 
g = 9.81;
u_e = [0;0;0;m*g];

xe = [0;0;-1;0;0;0;0;0;0;0;0;0;];

if isempty(Kout)
    syms o1 o2 o3 th1 th2 th3 v1 v2 v3 w1 w2 w3 u1 u2 u3 u4 real;

    xs = [o1 o2 o3 v1 v2 v3 th1 th2 th3 w1 w2 w3]';
    us = [u1 u2 u3 u4]';

    f = Quad_EOM(xs,us);

    
    eqn = Quad_EOM(xe,us) == zeros(12,1);
    sol = solve(eqn);
    ue = [double(sol.u1);
        double(sol.u2);
        double(sol.u3);
        double(sol.u4)];

    Ac = double(subs(jacobian(f,xs),[xs;us],[xe;ue]));
    Bc = double(subs(jacobian(f,us),[xs;us],[xe;ue]));


    Ad = expm(Ac*dt);
    fun = @(t) expm(Ac*t)*Bc;
    Bd = integral(fun,0,dt,'ArrayValue',true);

    Ad = Ad(1:6,1:6);
    Bd = Bd(1:6,:);
    
    Q = [500 0 0 0 0 0;
         0 500 0 0 0 0;
         0 0 500 0 0 0;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
     
    R = [5 0 0 0 ;
         0 5 0 0;
         0 0 5 0;
         0 0 0 5];
     
    Kout = dlqr(Ad, Bd, Q, R);
end

%%% RETURN OUTER LOOP CONTROL VECTOR
u = -Kout*(x(1:6)-xe(1:6))+u_e;
%}

