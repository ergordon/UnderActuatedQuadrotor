function target = Outer(t,x,dt)

g = 9.81;
m = .715;
persistent K ue
if isempty(K)
ue = [0;0;g];
wn = 1.5;
d = .8;
K = [-eye(3)*wn^2,-2*d*wn*eye(3)];
end

x_goal = [0;0;-1;0;0;0];
q=x(1:6);
a_des = K*(q-x_goal)+ue;
targetn = -((a_des-[0;0;g])*m)/norm(((a_des-[0;0;g])*m));
R = Rz(x(7))*Ry(x(8))*Rx(x(9)); 
% targetn = R^(-1)*targetn;
targetf = m*a_des(3);
target = [targetn;targetf];
% fprintf('%2.2f %2.2f %2.2f \n%2.2f %2.2f %2.2f \n \n',[targetn*targetf/m;x(1:3)])
% target = [0;0;-1;.714*9.81];
end

