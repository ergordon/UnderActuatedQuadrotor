function u=Outer(t,x,dt)
%%% NOMINAL CONTROL - This is the baseline control which we used when we
% linearized the system

persistent Kout Kx Ky Kz

dt = 0.01;
m = 0.715; 
g = 9.81;
u_e = [0;0;0;m*g];

xe = [0;0;-1;0;0;0;0;0;0;0;0;0;];

if isempty(Kx)
    %{
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
     
    Kout = dlqr(Ad, Bd, Q, R)
    %}
    %LQR CONTROLLER X
    dt = 0.001;
    A = [0,1;0,0];
    B = [0; -g];
    Q = [2,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;

    [P,E,Kx] = dare(A,B,Q,R);


    %LQR CONTROLLER Y
    dt = 0.001;
    A = [0,1;0,0];
    B = [0; g];
    Q = [2,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;
    [P,E,Ky] = dare(A,B,Q,R);



    %LQR CONTROLLER Z
    dt = 0.001;
    A = [0,1;0,0]; 
    B = [0; -1/m];
    Q = [5,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;
    [P,E,Kz]=dare(A,B,Q,R);
    
    Kx 
    Ky
    Kz
end
[x_des, y_des, z_des, yaw_des] = planner(x,t);

x1 = [x(1); x(4)];
a_x = -Kx*(x1-[x_des;0]);

x2 = [x(2); x(5)];
a_y = -Ky*(x2-[y_des;0]);

x3 = [x(3); x(6)];
delta_thrust = -Kz*(x3-[z_des;0]);

% YAW CONTROL
delta_yaw = yaw_des;
delta_pitch = (a_x*cos(x(9))-a_y*sin(x(9)));
delta_roll =  (a_x*sin(x(7))+a_y*cos(x(7)));

%delta_u = [delta_roll; delta_pitch; delta_yaw; delta_thrust];
delta_u = [delta_roll; delta_pitch; delta_yaw; delta_thrust];
%%% RETURN OUTER LOOP CONTROL VECTOR
u = delta_u + u_e;

%%% RETURN OUTER LOOP CONTROL VECTOR
%u = -Kout*(x(1:6)-xe(1:6))+u_e;
%}

