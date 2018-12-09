clc; clear; close all;
disp('started')
[t, o, theta, v, omega, u, odes] = lab3_simulate();
disp('finished')

%%
%uncomment these to keep the quadrotor in place so you can look at it spin
%  o(3,:) = -1;
%  o(2,:) = 0;
%  o(1,:) = 0;

close all;
moviefile = [];%['working_quad'];
lab3_visualize(t, o, theta, odes, moviefile)

%%
close all;
figure()
plot(t,o(1,:),t,o(2,:),t,o(3,:))
legend('x','y','z')
title('position')
% 
% figure()
% plot(t,v(1,:),t,v(2,:),t,v(3,:))
% legend('vx','vy','vz')
% title('velocity')

figure()
plot(t,theta(3,:).*180/pi,t,theta(2,:).*180/pi)%,t,theta(1,:))
legend('tx','ty')%,'tz')
title('angle')

% figure()
% plot(t,omega(1,:),t,omega(2,:),t,omega(3,:))
% legend('omegax','omegay','omegaz')
% title('angle rates')
% 
% figure()
% plot(t(1:end-1),u(1,:),t(1:end-1),u(2,:),t(1:end-1),u(3,:),t(1:end-1),u(4,:))
% legend('sig1','sig2','sig3','sig4')
% title('input forces')
% 
% 
% n = [];
% 
% for i=1:length(t)
%    n = [n omega(:,i)/norm(omega(:,i))]; 
% end
% 
% figure()
% plot(t,n(1,:),t,n(2,:),t,n(3,:))
% legend('nx','ny','nz')
% title('n')
% set(gcf,'paperorientation','landscape');
%     set(gca,'fontsize',18)
%     set(gcf,'paperunits','normalized');
%     set(gcf,'paperposition',[0 0 1 1]);
%     set(gca,'linewidth',1.5);
%     print(gcf,'-dpdf',"gains4");