function u=Inner(t,x,u_outer)
%
% x: state vector [x;y;z;vx;vy;vz;theta_3;theta_2;theta_1;p;q;r]
%
%         NOTE: theta_3 = roll, theta_2 = pitch, theta_1 = yaw
%
% 
% u_outer: [theta_3_star, theta_2_star, theta_1_star, u4_star] , 
%          where these values represent the "desired" or "nominal" 
%          roll, pitch, yaw, and thrust, respectively.
%
% The main purpose of the inner loop controller is to regulate attitude.
% We achieve this by linearizing about the level attitude (roll, pitch = 0),
% and design 3 independent LQR controllers for the roll, pitch, and yaw regulation.
%
% For now, we will simply pass through the nominal thrust given in u_outer.
%

% MEASUREMENT
th=angle_meas(x);  % roll, pitch, yaw == theta_3, theta_2, theta_1
w=gyro(x);

persistent Kroll Kpitch Kyaw
params = GetParameters;
dt = params.dt_inner;
J = params.J;

if isempty(Kroll)
    %LQR CONTROLLER ROLL(THETA3)
    A = [0,1;0,0];
    B = [0; 1/J(1,1)];
    Q = [2,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;

    [P,E,Kroll] = dare(A,B,Q,R);


    %LQR CONTROLLER PITCH(THETA2)
    A = [0,1;0,0];
    B = [0; 1/J(2,2)];
    Q = [2,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;
    [P,E,Kpitch] = dare(A,B,Q,R);



    %LQR CONTROLLER YAW(THETA1)
    A = [0,1;0,0]; 
    B = [0; 1/J(3,3)];
    Q = [5,0;0,.1];
    R = 5;

    A = eye(2) + dt*A;
    B = dt*B;
    [P,E,Kyaw]=dare(A,B,Q,R);
    
    Kroll
    Kyaw
    Kpitch
end

u = [0;0;0;0];

a = u_outer(3);
b = u_outer(2);
c = u_outer(1);

xroll = [th(1)-c; w(1)];
xpitch = [th(2)-b; w(2)];
xyaw = [th(3)-a; w(3)];

u(1) = -Kroll*xroll;
u(2) = -Kpitch*xpitch;
u(3) = -Kyaw*xyaw;
u(4) = u_outer(4);




