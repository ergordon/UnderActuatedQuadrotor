function u=Outer(t,x,dt,varargin)
%%% NOMINAL CONTROL - This is the baseline control which we used when we
% linearized the system

persistent Kout

m = 0.715; 
g = 9.81;
u_e = [0;0;0;m*g];

xe = planner(t,x);

p = inputParser;
addRequired(p,'t');
addRequired(p,'x');
addRequired(p,'dt');
addParameter(p,'Q',diag([500,500,500,1,1,1]))
addParameter(p,'R',diag([50,50,50,50]))
parse(p,t,x,dt,varargin{:})
t = p.Results.t;
x = p.Results.x;
dt = p.Results.dt;
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
    
    Q = p.Results.Q;
    R = p.Results.R;
     
    Kout = dlqr(Ad, Bd, Q, R)
end

%%% RETURN OUTER LOOP CONTROL VECTOR
u = -Kout*(x(1:6)-xe(1:6))+u_e;
%}

