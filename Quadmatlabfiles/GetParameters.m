function [params] = GetParameters(params)
%% Position Paramaters
params.odes = [0 0 0];     % oDesired
x0=rand(12,1)/10000; 
x0(1:3)=[0;0;0];          % Initial Position
x0(10)=0;
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
params.J = diag([4093e-6, 3944e-6, 7593e-6]); % Moment of Inertia
end