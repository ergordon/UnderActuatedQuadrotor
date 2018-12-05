function u=Outer(t,x,dt,desired_x)

persistent K1 K2 K3
%%% MEASUREMENT
x=MoCap(x); % full state [x;y;z;vx;vy;vz;theta_3;theta_2;theta_1;p;q;r]

%%% NOMINAL CONTROL - This is the baseline control which we used when we
% linearized the system
m=.6891;
g=9.81;
u_nom=[0;0;0;m*g];

%%% FEEDBACK CONTROL
% POSITION CONTROL
delta_theta3_des=0;
delta_theta2_des=0;
delta_u4=m*g;

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
    
    Kx
    Ky
    Kz
end

x1=[x(1)-desired_x(1); x(4)];
% delta_theta2_des=-K1*x1;
ax = -Kx*x1;

x2=[x(2)-desired_x(2); x(5)];
% delta_theta3_des=-K2*x2;
ay = -Ky*x2;

yaw = x(9);
delta_theta2_des = ax*cos(yaw) - ay*sin(yaw);
delta_theta3_des = ax*sin(yaw) + ay*cos(yaw);

x3=[x(3)-desired_x(3); x(6)];
delta_u4=-Kz*x3;

% YAW CONTROL
delta_theta1_des=desired_x(4);

delta_u=[delta_theta3_des;delta_theta2_des;delta_theta1_des;delta_u4];

%%% RETURN OUTER LOOP CONTROL VECTOR
u=u_nom + delta_u;