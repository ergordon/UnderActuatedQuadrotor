function xe = planner(t, x)

q = x(1:3);
b_att = 3;
k_att = 3;
b_des = 1;
k_des = 1;


    q_goal = [sin(t/3);cos(t/3);-1];


if norm(q-q_goal) <= b_att
    gradf = k_att*(q-q_goal);
else
    gradf = b_att*k_att*(q-q_goal)/norm((q-q_goal));
end

if  norm(k_des*gradf) <= b_des
    q = q - k_des*gradf;
else
    q = q - b_des*(gradf/norm(gradf));
end

xe=zeros(12,1);
xe(1:3) = q;
% fprintf('%2.5f \t%2.5f \t%2.5f \t%2.5f\n',t,xe(1),xe(2),xe(3))

