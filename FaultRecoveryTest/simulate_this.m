function [params] = simulate_this()
close all
clear all
clc

% Fetch Parameters
params = GetParameters;
o_des = params.odes;
dt_inner = params.dt_inner;
dt_outer = params.dt_outer;
t0 = params.t0;
t1 = params.t1;
x0 = params.x0;


% Initial conditions
tspan=[t0 t1]; 

% Identify your EOM function, policy functions and a force disturbance. 
eom=@(t,x,u) Quad_EOM(x,u);
inner_policy=@(t,x,u) Inner(t,x,u);
outer_policy=@(t,x) Outer(t,x,dt_outer);

%%% Discretize policy, run simulation.
discrete_policy=@(t,x) discretize(inner_policy,outer_policy,tspan,dt_inner,dt_outer,t,x);
sys=@(t,x) GetBoundedInputs(eom,discrete_policy,t,x);
options=odeset('RelTol',1e-5,'MaxStep',min(dt_inner,dt_outer));
tic; 
[T,X]=ode45(sys,tspan,x0,options);
toc
[Ui_T,Ui,Uo_T,Uo]=discretize('Get Controller History');

odes = [];
for i=1:length(T)
    odes = [odes; o_des];
end

%figure()
Visualizer(T', X(:,1:3)', X(:,7:9)', odes', 'Test.mp4')
close all;

% Plot results
plot_basic(T,X',Ui_T,Ui,Uo_T,Uo)

% Clear persistent variables
clear functions