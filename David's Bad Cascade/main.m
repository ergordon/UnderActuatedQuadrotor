clc; clear; close all;

[t, state] = lab3_simulate();


o = state(1:3,:);
theta = state(4:6,:);
odes = [zeros(2,length(t));
        -1.*ones(1,length(t))];
moviefile = 'please_work.mp4';
lab3_visualize(t, o, theta, odes, moviefile)

%%
close all;

figure()
h = plot(o(1,:),o(2,:),'b-', odes(1,:),odes(2,:),'r--');
set(h(1),'linewidth',2)
set(h(2),'linewidth',2)
legend('actual path','desired path');
ylabel('Position [m]');
xlabel('Position [m]');
title('Top-down View of Quadrotor Path')
set(gcf,'paperorientation','landscape');
    set(gca,'fontsize',18)
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    set(gca,'linewidth',1.5);


  
figure()
h = plot(t,o(1,:),'r-',t,o(2,:),'b-',t,o(3,:),'g-');
set(h(1),'linewidth',2)
set(h(2),'linewidth',2)
set(h(3),'linewidth',2)
hold on
h = plot(t(1:end),odes(1,:),'r--',t(1:end),odes(2,:),'b--',t(1:end),odes(3,:),'g--');
set(h(1),'linewidth',2)
set(h(2),'linewidth',2)
set(h(3),'linewidth',2)
legend('x','y','z','x_{desired}','y_{desired}','z_{desired}');
ylabel('Position [m]');
xlabel('Time [s]');
title('Quadrotor Position vs Time')
set(gcf,'paperorientation','landscape');
    set(gca,'fontsize',18)
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    set(gca,'linewidth',1.5);

 theta_des = [0;0];
 t_des = [0;10];
figure()
h = plot(t,theta(1,:),'r-',t_des,theta_des,'r--');
set(h(1),'linewidth',2)
set(h(2),'linewidth',2)
legend('\theta_z','\theta_{z}_{desired}');
ylabel('Yaw [rad]');
xlabel('Time [s]');
title('Quadrotor Yaw vs Time')
set(gcf,'paperorientation','landscape');
    set(gca,'fontsize',18)
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    set(gca,'linewidth',1.5);
 
    

