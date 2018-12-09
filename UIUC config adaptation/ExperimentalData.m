function ExperimentalData
close all; clc; clear;
data=csvread('ExperimentalData4.csv',1,0);
t = data(:,1)
x = 0*t;
y = 0*t;
z = 1+0*t;
plot(t,data(:,2),t,data(:,3),t,-data(:,4),'linewidth',1)
hold on
plot(t,x,'--k',t,y,'--k',t,z,'--k','linewidth', 1)

legend('X Actual','Y Actual','Z Actual','X Desired','Y Desired','Z Desired')

err2=[(data(:,2)-x).^2,(data(:,3)-y).^2,(data(:,4)-x).^2];
sum(err2,1)

end