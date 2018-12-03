function u=Outer(t,x,dt)
%%% NOMINAL CONTROL - This is the baseline control which we used when we
% linearized the system

persistent Kout Kx Ky Kz
params = GetParameters;
dt_outer = params.dt_outer;
m = params.m;
g = params.g;

u_e = [0;0;0;m*g];

if isempty(Kx)
    %LQR CONTROLLER X
    dt = dt_outer;
    A = [0,1;0,0];
    B = [0; -g];
    Q = [2,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;

    [P,E,Kx] = dare(A,B,Q,R);


    %LQR CONTROLLER Y
    dt = dt_outer;
    A = [0,1;0,0];
    B = [0; g];
    Q = [2,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;
    [P,E,Ky] = dare(A,B,Q,R);



    %LQR CONTROLLER Z
    dt = dt_outer;
    A = [0,1;0,0]; 
    B = [0; -1/m];
    Q = [5,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;
    [P,E,Kz]=dare(A,B,Q,R);
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

