function plot_basic(t, x, Ui_T,Ui,Uo_T,Uo)
figure(1)
hold on
plot(t,x(1,:),'b-');
plot(t,x(2,:),'g-');
plot(t,x(3,:),'r-');
title('Positions')
figure(2)
plot(t,x(4:6,:));
title('Velocities')
figure(3)
plot(t,x(7:9,:))
title('Angles')
figure(4)
plot(t,x(10:12,:))
title('Angular Rates')
figure(6)
plot(Ui_T,Ui)
legend('u_1', 'u_2', 'u_3', 'u_4')
title('Inner Loop Control Inputs')
figure(7)
plot(Uo_T,Uo)
legend('u_1', 'u_2', 'u_3', 'u_4')
title('Outer Loop Control Inputs')