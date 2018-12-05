close all
clear all

x0 = [0; 0; 0; 0; 0; 0];
goal = @(t) [1; 1; 1; 0; 0; 0];
t1 = 10;
dt = .02;

x = [x0];
t = 0;
desiredstate = [1; 1; 1; 0; 0; 0];
wn = 1.5;
d = .8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% outline for outer loops stuffs
% Pass in x t
% if empty persist k and ue for runtime
% x_goal = planner(x,t) NEEDS TO OUTPUT 6x1!! pos,vel
% 
% K = [-eye(3)*wn^2,-2*d*wn*eye(3)];
% a_des = K*(x-x_goal)+ue;
% targetn = (a_des*m)/norm(a_des*m);
% targetf = (a_des(3)-norm(g))*m/targetn(3);
% output [targets]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Testing the positional controlller to see if a_des=ui is correctly chosen
for i = 1:t1/dt
xi = x(:,end);
ti = t(:,end);
ue = [0;0;9.81];
x_goal = goal(i*dt);
ui = K*(xi-x_goal)+ue;

[T,X] = ode45(@(ti,xi) eom(ti,xi,ui),[ti,ti+dt],xi);
x = [x,X(end,:)'];
t = [t,T(end)];
desiredstate = [desiredstate,x_goal];
end
plot(t,x(1:3,:),t,desiredstate(1:3,:))
legend('actx','acty','actz','desx','desy','desz')

function xdot = eom(t,x,u)
persistent A B
if isempty(A)
    A=[zeros(3),eye(3);zeros(3),zeros(3)];
    B=[zeros(3);eye(3)];
end
xdot = A*x+B*u+[0;0;0;0;0;-9.81];
end