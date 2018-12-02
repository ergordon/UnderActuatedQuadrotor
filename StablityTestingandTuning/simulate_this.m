% Initial conditions
function [T, X, Ui_T, Ui, Uo_T, Uo, odes] = simulate_this(varargin)
dt = (1/50);
t0 = 0;
t1 = 25;

tspan=[t0 t1]; 
dt_inner=0.002; 
dt_outer=0.02;
x0=zeros(12,1); 
x0(1:3)=[0;0;0];
% x0(7:9)=[0.1;0.1;.1];

p = inputParser;
addParameter(p,'Qo',diag([2500,2500,500,1,1,1]))
addParameter(p,'Ro',diag([10,10,10,10]))
addParameter(p,'Qi',diag([250,250,250,1,1,1]))
addParameter(p,'Ri',diag([1,1,1,1]))
parse(p,varargin{:})
Qo = p.Results.Qo;
Ro = p.Results.Ro;
Qi = p.Results.Qi;
Ri = p.Results.Ri;

% Identify your EOM function, policy functions and a force disturbance. 
eom=@(t,x,u) Quad_EOM(x,u);
inner_policy=@(t,x,u) Inner(t,x,u,'Q',Qi,'R',Ri);
outer_policy=@(t,x) Outer(t,x,dt_outer,'Q',Qo,'R',Ro);

%%% Discretize policy, run simulation.
discrete_policy=@(t,x) discretize(inner_policy,outer_policy,tspan,dt_inner,dt_outer,t,x);
sys=@(t,x) GetBoundedInputs(eom,discrete_policy,t,x);
options=odeset('RelTol',1e-5,'MaxStep',min(dt_inner,dt_outer));
tic; 
[T,X]=ode45(sys,tspan,x0,options);
toc
[Ui_T,Ui,Uo_T,Uo]=discretize('Get Controller History');

odes = repmat([0 0 -1], [length(T),1]);

%figure()
% Visualizer(T', X(:,1:3)', X(:,7:9)', odes', 'Test.mp4')
% close all;

% Plot results
% plot_basic(T,X',Ui_T,Ui,Uo_T,Uo)

% Clear persistent variables
clear functions
end