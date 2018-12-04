function [params] = GetParameters(params)
%% Position Paramaters
params.odes = [0 0 -1];     % oDesired
x0=zeros(12,1); 
x0(1:3)=[0;-2;-1];           % Initial Position
params.x0 = x0;         
params.goal = [2;0;-2];     % Goal Position

%% Time Parameters
params.dt_inner=0.001;      % Inner Loop dt
params.dt_outer=0.01;       % Outer Loop dt
params.t0 = 0;              % Start Time
params.t1 = 10;              % End Time

%% QuadRotor Parameters
params.m = 0.715;           % Mass
params.g = 9.81;            % Gravity Accl
params.JT = diag([3.2e-3 3.2e-3 5.5e-3]);
params.JP = diag([0 0 1.5e-5]);
params.JB = params.JT - 4.*params.JP;
params.kF = 6.41e-6;
params.kT = 1.69e-2;
params.gamma = 2.75e-3;
end