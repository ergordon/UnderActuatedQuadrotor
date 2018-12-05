function f=Quad_EOM(x,u)
%
% x: state vector [x;y;z;vx;vy;vz;theta_3;theta_2;theta_1;p;q;r]
%
%         NOTE: theta_3 = roll, theta_2 = pitch, theta_1 = yaw
%
% u: control vector [u1, u2, u3, u4] == roll, pitch, and yaw torque, and thrust
%

% Physical Parameters
J = [0.004029, 0, 0; 0, 0.004015, 0; 0, 0, 0.007593];
g = 9.81;  % gravity
m = 0.6891;

f=zeros(12,1);

vx=x(4);
vy=x(5);
vz=x(6);

theta_3=x(7);
theta_2=x(8);
theta_1=x(9);

p=x(10);
q=x(11);
r=x(12);

f(1)=vx;
f(2)=vy;
f(3)=vz;

u1=u(1);
u2=u(2);
u3=u(3);
u4=u(4);

f(4)=(-(cos(theta_1)*sin(theta_2)*cos(theta_3)+sin(theta_1)*sin(theta_3))*u4)/m;
f(5)=(-(sin(theta_1)*sin(theta_2)*cos(theta_3)-cos(theta_1)*sin(theta_3))*u4)/m;
f(6)=(m*g-(cos(theta_2)*cos(theta_3)*u4))/m;

% p=x(10);
% q=x(11);
% r=x(12);
% 
f(10:12)=inv(J)*([u1; u2; u3]-[0,-r, q; r, 0, -p; -q, p, 0]*J*[p; q; r]); 

% f(12) = (1/J(1,1))*(u1 + (J(2,2)-J(1,1))*q*p);
% f(11) = (1/J(2,2))*(u2 + (J(3,3)-J(1,1))*r*p);
% f(10) = (1/J(3,3))*(u3 + (J(1,1)-J(2,2))*r*q);
% 
m=[0, sin(theta_3)/cos(theta_2), cos(theta_3)/cos(theta_2); 0, cos(theta_3), -sin(theta_3);...
           1, tan(theta_2)*sin(theta_3), tan(theta_2)*cos(theta_3)]*[p;q;r];

f(7)=m(3);
f(8)=m(2);
f(9)=m(1);

   


% Compute state vector derivative

