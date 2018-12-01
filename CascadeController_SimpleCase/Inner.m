function u=Inner(t,x,u_outer)
% The main purpose of the inner loop controller is to regulate attitude.
% We achieve this by linearizing about the level attitude (roll, pitch = 0),
% and design 3 independent LQR controllers for the roll, pitch, and yaw regulation.
%
% For now, we will simply pass through the nominal thrust given in u_outer.
%

%%% FEEDBACK CONTROL
% POSITION CONTROL
%%{
persistent K
dt = 0.001;
th = x(7:9); 
w = x(10:12);
J = diag([4093e-6, 3944e-6, 7593e-6]);

xe = [0;0;-1;0;0;0;0;0;0;0;0;0;];

if isempty(K)
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

    Ad = Ad(7:12,7:12);
    Bd = Bd(7:12,:);
    
    Q = [250 0 0 0 0 0;
         0 250 0 0 0 0;
         0 0 500 0 0 0;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
     
        R = [5 0 0 0 ;
         0 5 0 0;
         0 0 5 0;
         0 0 0 5];
     
    K = dlqr(Ad, Bd, Q, R);
end

u = -K*(x(7:12)-xe(7:12))+u_outer;

%}