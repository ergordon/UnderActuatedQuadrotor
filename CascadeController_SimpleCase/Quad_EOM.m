function xdot=Quad_EOM(x,u)

%
% x: state vector [x;y;z;vx;vy;vz;theta_1;theta_2;theta_3;p;q;r]
%
%         NOTE: theta_1 = roll, theta_2 = pitch, theta_3 = yaw

%% Physical Parameters
J = diag([4093e-6, 3944e-6, 7593e-6]);    % moment of inertia matrix in body frame
g = 9.81;  % gravity
m = 0.715;                  % mass

o = x(1:3);
v = x(4:6);
t = x(7:9);
w = x(10:12);

what = [0 -w(3) w(2);
        w(3) 0 -w(1);
        -w(2) w(1) 0];
%ZYX
R = Rz(t(1))*Ry(t(2))*Rx(t(3));
N = [Rx(t(3))'*Ry(t(2))'*[0;0;1] Rx(t(3))'*[0;1;0] [1;0;0]]^(-1);

odot = v;
tdot = N*w;
vdot = 1/m*([0;0;m*g] + R*[0;0;-u(4)]);
wdot = J^(-1)*(u(1:3) - what*J*w);

xdot = [odot;vdot;tdot;wdot];
end

function R = Rz(t)
    R = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
end

function R = Ry(t)
    R = [cos(t) 0  sin(t); 0 1 0; -sin(t) 0 cos(t)];
end

function R = Rx(t)
    R = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
end
