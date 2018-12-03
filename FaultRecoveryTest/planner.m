
function [x_des, y_des, z_des, yaw_des] = planner(x,t)
%% Planner
current = [x(1); x(2); x(3)];
params = GetParameters;
goal = params.goal;
tgoal = params.t1;
for i = 1:3
    if abs(current(i)-goal(i))>0.5
        des(i) = current(i)+(goal(i)-current(i))*(t/tgoal);
    else
        des(i)= goal(i);
    end
end
x_des = des(1);
y_des = des(2);
z_des = des(3);
yaw_des = 0;
