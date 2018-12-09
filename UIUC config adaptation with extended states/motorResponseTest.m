clc; clear; close all;





u = 10;
xo = 0;

tspan = [0 .1];

[t,x] = ode45(@(t,x) getRates(t,x,u),tspan,xo);

plot(t,x)




function xdot = getRates(t,x,u)

    sigma = .015;
    
    xdot = 1/sigma*(u-x);



end