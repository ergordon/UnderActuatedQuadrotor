function xdot=Quad_EOM(x,u)

%
% x: state vector [x;y;z;vx;vy;vz;theta_1;theta_2;theta_3;p;q;r]
%
%         NOTE: theta_1 = roll, theta_2 = pitch, theta_3 = yaw

%% Physical Parameters
params = GetParameters;
J = params.J;
g = params.g;
m = params.m;

xdot=zeros(12,1);

vx=x(4);
vy=x(5);
vz=x(6);

theta_3=x(7);
theta_2=x(8);
theta_1=x(9);

p=x(10);
q=x(11);
r=x(12);

xdot(1)=vx;
xdot(2)=vy;
xdot(3)=vz;

u1=u(1);
u2=u(2);
u3=u(3);
u4=u(4);

xdot(4)=(-(cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*u4)/m;
xdot(5)=(-(sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*u4)/m;
xdot(6)=(m*g-(cos(theta_2)*cos(theta_3)*u4))/m;

xdot(10:12)=inv(J)*([u1; u2; u3]-[0,-r, q; r, 0, -p; -q, p, 0]*J*[p; q; r]);

m=[0, sin(theta_3)/cos(theta_2), cos(theta_3)/cos(theta_2); 0, cos(theta_3), -sin(theta_3);...
           1, tan(theta_2)*sin(theta_3), tan(theta_2)*cos(theta_3)]*[p;q;r];
       
xdot(7)=m(3);
xdot(8)=m(2);
xdot(9)=m(1);