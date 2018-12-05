clearvars -except state_x desired_x t_vector X_all T_all timer new_check obst_all des_all

% Initial conditions
tspan = [t_vector(1) t_vector(2)];
dt_inner=0.001; dt_outer=0.001;
x0 = state_x;

% Identify your EOM function, policy functions and a force disturbance. 
eom=@(t,x,u) Quad_EOM(x,u);
inner_policy=@(t,x,u) Inner(t,x,u);
outer_policy=@(t,x) Outer(t,x,dt_outer,desired_x);

% Default force disturbance is the identity (no disturbance)
disturbance=@(t,f)f;

%%% Discretize policy, run simulation. You shouldn't need to touch this
%%% section.
discrete_policy=@(t,x) discretize(inner_policy,outer_policy,tspan,dt_inner,dt_outer,t,x);
sys=@(t,x)vect12auto4(eom,discrete_policy,disturbance,t,x);
options=odeset('RelTol',1e-5,'MaxStep',min(dt_inner,dt_outer));
tic; [T,X]=ode45(sys,tspan,x0,options); toc
[Ui_T,Ui,Uo_T,Uo]=discretize('Get Control History');
%%%

% Plot results
%plot_basic 

% Clear persistent variables
%clear functions