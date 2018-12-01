clc; clear; close all;

[t, o, theta, omega, odes] = lab3_simulate();

%%
close all;
% o(3,:) = -1;
% o(2,:) = 0;
% o(1,:) = 0;
lab3_visualize(t, o, theta, odes, [])

%%
figure()
plot(t,o(1,:),t,o(2,:),t,o(3,:))
legend('x','y','z')
title('position')

figure()
plot(t,theta(1,:),t,theta(2,:),t,theta(3,:))
legend('tx','ty','tz')
title('angle')

figure()
plot(t,omega(1,:),t,omega(2,:),t,omega(3,:))
legend('omegax','omegay','omegaz')
title('angle rates')

n = [];

for i=1:length(t)
   n = [n omega(:,i)/norm(omega(:,i))]; 
end

figure()
plot(t,n(1,:),t,n(2,:),t,n(3,:))
legend('nx','ny','nz')
title('n')