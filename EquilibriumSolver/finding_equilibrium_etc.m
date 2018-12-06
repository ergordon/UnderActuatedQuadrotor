clc; clear; close all;

%parameters
JT = diag([4093e-6 3944e-6 7593e-6]);
JP = diag([0 0 1.5e-11]);
JB = JT - 4.*JP;

g = 9.81;
l = .17;
m = 0.715;
kF = 7.46e-6;
kM = 1.23e-7;
kT = kF/kM;
gamma = 2.75e-3;

% syms m g l gamma JBx JBy JBz JPx JPy JPz gamma kF kT real
% JB = diag([JBx JBy JBz]);
% JP = diag([JPx JPy JPz]);
% JT = JB + 4*JP;



syms nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4 w1dot w2dot w3dot w4dot real
assume(w1 > 0)
assume(w2 > 0)
assume(w3 > 0)
% assume(w4 > 0)
% assume(nx > 0)
% assume(ny > 0)
% assume(nz > 0)


wB = [p; q; r];
% wBdot = [pdot; qdot; rdot];

wP1dot = [0; 0; w1dot];
wP2dot = [0; 0; w2dot];
wP3dot = [0; 0; w3dot];
wP4dot = [0; 0; w4dot];

wP1 = [0; 0; w1];
wP2 = [0; 0; w2];
wP3 = [0; 0; w3];
% wP4 = [0 0 w4]';
wP4 = [0; 0; 0];

f1 = kF*w1^2;
f2 = kF*w2^2;
f3 = kF*w3^2;
f4 = 0 ;%kF*w4^2;

t1 = kT*f1;
t2 = kT*f2;
t3 = -kT*f3;
t4 = 0;%-kT*f4;

td = [0; 0; -gamma*r];

% T12 = JB*wBdot;% + JP*(wP1dot+wP2dot+wP3dot+wP4dot);
T3 = cross(wB,JT*wB + JP*(wP1+wP2+wP3));%+wP4));
Tres = [(f1-f2)*l+td(1);
        (f3-f4)*l+td(2);
         t1+t2+t3+t4+td(3)];

eq123 =  Tres == T3;% + T12;

n = [nx; ny; nz];

eq456 = n == epsilon*wB;
eq7 = norm(epsilon*wB) == 1;
% eq456 = n == wB/norm(wB);
% eq7 = norm(n) == 1;


eq8 =  m*g == nz*(f1+f2+f3);%+f4);

%one prop failed
rho = 0.10;

% %two prop failed
% rho = 0;

% eq9 = f4 == 0;
eq10 = f1 == f2;

% eq121314 = wBdot == [0 0 0]';


eq11 = f3/f1 == rho;

sol = solve([eq123 eq456 eq7 eq8 eq10 eq11], [nx ny nz epsilon p q r w1 w2 w3]);

 

nx_eq = double(sol.nx)
ny_eq = double(sol.ny)
nz_eq = double(sol.nz)
epsilon_eq = double(sol.epsilon)
p_eq = double(sol.p)
q_eq = double(sol.q)
r_eq = double(sol.r)
% pdot_eq = double(sol.pdot)
% qdot_eq = double(sol.qdot)
% rdot_eq = double(sol.rdot)
w1_eq = double(sol.w1)
w2_eq = double(sol.w2)
w3_eq = double(sol.w3) 
% w4_eq = double(sol.w4)

f1_eq = kF*double(sol.w1)^2
f2_eq = kF*double(sol.w2)^2
f3_eq = kF*double(sol.w3)^2
% f4_eq = kF*double(sol.w4)^2

rho_eq = rho

% save equilib.mat nx_eq ny_eq nz_eq epsilon_eq p_eq q_eq r_eq pdot_eq qdot_eq rdot_eq w1_eq w2_eq w3_eq w4_eq rho_eq f1_eq f2_eq f3_eq f4_eq
%%
clc; clear; close all;

%parameters
JT = diag([4093e-6 3944e-6 7593e-6]);
JP = diag([0 0 1.5e-5]);
JB = JT - 4.*JP;

g = 9.81;
l = .17;
m = 0.715;
kF = 7.46e-6;
kM = 1.23e-7;
kT = kF/kM;
gamma = 2.75e-3;

fun = @equilibRoot;
var_guess = [0 .289 .958 0 5.69 18.89 400 400 200 .03]';

% options=optimset('disp','iter','LargeScale','off','TolFun',.000001,'MaxIter',100000,'MaxFunEvals',100000);
% variables = [nx ny nz p q r w1 w2 w3 epsilon]
disp('hello')
format long
vars = fsolve(fun, var_guess)


% lb = zeros([0 0 0 -50 -50 -50 0 0 0 0]); %lower bound
% ub = zeros([1 1 1 50 50 50 1000 1000 100 1]); %lower bound
% vars = lsqnonlin(fun,var_guess,lb)%,ub)
% vars = lsqnonlin(fun,var_guess)


% syms m g l gamma JBx JBy JBz JPx JPy JPz gamma kF kT real
% JB = diag([JBx JBy JBz]);
% JP = diag([JPx JPy JPz]);
% JT = JB + 4*JP;



% syms nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4 w1dot w2dot w3dot w4dot real
% assume(w1 > 0)
% assume(w2 > 0)
% assume(w3 > 0)
% assume(w4 > 0)
% assume(nx > 0)
% assume(ny > 0)
% assume(nz > 0)




 

% nx_eq = double(sol.nx)
% ny_eq = double(sol.ny)
% nz_eq = double(sol.nz)
% epsilon_eq = double(sol.epsilon)
% p_eq = double(sol.p)
% q_eq = double(sol.q)
% r_eq = double(sol.r)
% % pdot_eq = double(sol.pdot)
% % qdot_eq = double(sol.qdot)
% % rdot_eq = double(sol.rdot)
% w1_eq = double(sol.w1)
% w2_eq = double(sol.w2)
% w3_eq = double(sol.w3)
% % w4_eq = double(sol.w4)
% 
% f1_eq = kF*double(sol.w1)^2
% f2_eq = kF*double(sol.w2)^2
% f3_eq = kF*double(sol.w3)^2
% % f4_eq = kF*double(sol.w4)^2
% 
% rho_eq = rho

% save equilib.mat nx_eq ny_eq nz_eq epsilon_eq p_eq q_eq r_eq pdot_eq qdot_eq rdot_eq w1_eq w2_eq w3_eq w4_eq rho_eq f1_eq f2_eq f3_eq f4_eq



%%

clc; clear; close all;
load('equilib.mat');


neq = [nx; ny; nz];
wBeq = [p; q; r];
wBdoteq = [pdot; qdot; rdot];
w1eq = w1;
w2eq = w2;
w3eq = w3;
w4eq = w4;
epsiloneq = epsilon;
rho = 0.5;


syms m g l gamma JBx JBy JBz JPx JPy JPz gamma kF kT real
syms nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4 w1dot w2dot w3dot w4dot real

fs = GetReducedRates(nx, ny, nz, p, q, r, w1, w2, w3, w4, m, g, l, gamma, JBx, JBy, JBz, JPx, JPy, JPz, kF, kT)
ss = [p; q; nx; ny];


JT = diag([3.2e-3 3.2e-3 5.5e-3]);
JP = diag([0 0 1.5e-5]);
JB = JT - 4.*JP;

g = 9.81;
lr = .17;
m = 0.5;
kFr = 6.41e-6;
kT = 1.69e-2;
gamma = 2.75e-3;

% Asym = jacobian(fs,ss)
A = double(subs(jacobian(fs,ss),[nx;ny;nz;p;q;r;w1;w2;w3;w4;epsilon;JBx;JBy;JBz;JPx;JPy;JPz;kF;l],[neq;wBeq;w1eq;w2eq;w3eq;w4eq;epsiloneq;JB(1,1);JB(2,2);JB(3,3);JP(1,1);JP(2,2);JP(3,3);kFr;lr]));

us = [w1 w2 w3 w4]';

B = double(subs(jacobian(fs,us),[nx;ny;nz;p;q;r;w1;w2;w3;w4;epsilon;JBx;JBy;JBz;JPx;JPy;JPz;kF;l],[neq;wBeq;w1eq;w2eq;w3eq;w4eq;epsiloneq;JB(1,1);JB(2,2);JB(3,3);JP(1,1);JP(2,2);JP(3,3);kFr;lr]));

R = eye(size(B,2));
Q = eye(size(A));
K = lqr(A, B, Q, R)

%%
clc; clear; close all;
load('equilib.mat');

%parameters
JT = diag([3.2e-3 3.2e-3 5.5e-3]);
JP = diag([0 0 1.5e-5]);
JB = JT - 4.*JP;

g = 9.81;
l = .17;
m = 0.5;
kF = 6.41e-6;
kT = 1.69e-2;
gamma = 2.75e-3;


wB_eq = [p_eq q_eq r_eq]';
n_eq = wB_eq/norm(wB_eq); 


s_eq = [p_eq q_eq n_eq(1) n_eq(2)]';
u_eq = [f1_eq-f3_eq f2_eq]';


w1_eq = sqrt(f1_eq/kF);
w2_eq = sqrt(f2_eq/kF);
w3_eq = sqrt(f3_eq/kF);
w4_eq = 0;

a_eq = ( JT(1,1)-JT(3,3) )/JB(1,1)*wB_eq(3) - JP(3,3)/JB(1,1)*(w1_eq+w2_eq+w3_eq+w4_eq);

A = [0 a_eq 0 0;
     -a_eq 0 0 0;
     0 -n_eq(3) 0 wB_eq(3);
     n_eq(3) 0 -wB_eq(3) 0];
B = l/JB(1,1).*[0 1;
                1 0;
                0 0;
                0 0];

Q = diag([1 1 20 20]);
R = diag([1 1]);

K = lqr(A, B, Q, R)


%%
%lets try their equilibrium again


clc; clear; close all;
% load('equilib.mat');

%parameters
JT = diag([3.2e-3 3.2e-3 5.5e-3]);
JP = diag([0 0 1.5e-5]);
JB = JT - 4.*JP;

g = 9.81;
l = .17;
m = 0.5;
kF = 6.41e-6;
kT = 1.69e-2;
gamma = 2.75e-3;


f1_eq = 2.05;
f2_eq = 1.02;
f3_eq = 2.05;
f4_eq = 0;

p_eq = 0;
q_eq = 5.69;
r_eq = 18.89;
nx_eq = 0;
ny_eq = .289;
nz_eq = .958;

w1_eq = sqrt(f1_eq/kF);
w2_eq = sqrt(f2_eq/kF);
w3_eq = sqrt(f3_eq/kF);
w4_eq = sqrt(f4_eq/kF);

rho_eq = 0.5;



save('equilib.mat')

%%

load('equilib.mat')

wB_eq = [p_eq q_eq r_eq]';
n_eq = wB_eq/norm(wB_eq); 


s_eq = [p_eq q_eq n_eq(1) n_eq(2)]';
u_eq = [f1_eq-f3_eq f2_eq]';


% w1_eq = sqrt(f1_eq/kF);
% w2_eq = sqrt(f2_eq/kF);
% w3_eq = sqrt(f3_eq/kF);
% w4_eq = 0;

a_eq = ( JT(1,1)-JT(3,3) )/JB(1,1)*wB_eq(3) - JP(3,3)/JB(1,1)*(w1_eq+w2_eq+w3_eq+w4_eq);

A = [0 a_eq 0 0;
     -a_eq 0 0 0;
     0 -n_eq(3) 0 wB_eq(3);
     n_eq(3) 0 -wB_eq(3) 0];
B = l/JB(1,1).*[0 1;
                1 0;
                0 0;
                0 0];

Q = diag([1 1 20 20]);
R = diag([1 1]);

K = lqr(A, B, Q, R)


%%
clc; clear ; close all;

%let's check to see that their equilibrium is actually an equilibrium


%parameters
JT = diag([3.2e-3 3.2e-3 5.5e-3]);
JP = diag([0 0 1.5e-5]);
JB = JT - 4.*JP; 

g = 9.81;
l = .17;
m = 0.5;
kF = 6.41e-6;
kT = 1.69e-2;
gamma = 2.75e-3;

% syms m g l gamma JBx JBy JBz JPx JPy JPz gamma kF kT real
% JB = diag([JBx JBy JBz]);
% JP = diag([JPx JPy JPz]);



% syms nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4 w1dot w2dot w3dot w4dot real


f1_eq = 2.05;
f2_eq = 1.02;
f3_eq = 2.05;
f4_eq = 0;

w1_eq = sqrt(f1_eq/kF);
w2_eq = sqrt(f2_eq/kF);
w3_eq = sqrt(f3_eq/kF);
w4_eq = sqrt(f4_eq/kF);

rho_eq = 0.5;

p_eq = 0;
q_eq = 5.69;
r_eq = 18.89;
nx_eq = 0;
ny_eq = 0.289;
nz_eq = .958;

pdot_eq = 0;
qdot_eq = 0;
rdot_eq = 0;


wB = [p_eq q_eq r_eq]';
wBdot = [pdot_eq qdot_eq rdot_eq]';

wP1 = [0 0 w1_eq]';
wP2 = [0 0 w2_eq]';
wP3 = [0 0 w3_eq]';
wP4 = [0 0 w4_eq]';


f1 = kF*w1_eq^2;
f2 = kF*w2_eq^2;
f3 = kF*w3_eq^2;
f4 = kF*w4_eq^2;

t1 = (-1)^(1+1)*kT*f1;
t2 = (-1)^(2+1)*kT*f2;
t3 = (-1)^(3+1)*kT*f3;
t4 = (-1)^(4+1)*kT*f4;

td = [0 0 -gamma*r_eq]';

% T12 = JB*wBdot% + JP*(wP1dot+wP2dot+wP3dot+wP4dot);
% % % T3 = cross(wB,(JB+4*JP)*wB + JP*(wP1+wP2+wP3+wP4));
% T3 = cross(wB,JB*wB + JP*(wB+wP1) + JP*(wB+wP2) + JP*(wB+wP3) + JP*(wB+wP4));

wsum = w1_eq + w2_eq + w3_eq + w4_eq;

seven = [(JT(3,3) - JT(2,2))*q_eq*r_eq + JP(3,3)*q_eq*wsum;
         (JT(3,3) - JT(1,1))*p_eq*r_eq + JP(3,3)*p_eq*wsum;
         (JT(2,2) - JT(1,1))*p_eq*q_eq]


Tres = [(f2-f4)*l+td(1);
        (f3-f1)*l+td(2);
         t1+t2+t3+t4+td(3)];


n = [nx_eq ny_eq nz_eq]';
rho = rho_eq;

%nope!
% eq123 = 
% T12 + T3 
Tres

%yep
% eq456 = 
% n 
% wB/norm(wB)

%yep
%eq8 = 
% (f1+f2+f3+f4)*nz_eq
% m*g

%yep
% eq9 = 
% f4 
% 0

%yep
% eq10 = 
% f1 
% f3

% eq121314 = 
% wBdot 
% [0 0 0]'

%yep
% eq11 = 
% f2/f1 
% rho

% eqs = [eq123;eq456;eq7;eq8;eq9;eq10;eq11;eq121314]


% R = sqrt(1-n(3)^2)/n(3)*9.81/norm(wB)^2

kF*(w2_eq^2 - w4_eq^2)*l - (JT(3,3) - JT(1,1))*q_eq*r_eq - JP(3,3)*q_eq*wsum

kF*(w3_eq^2 - w1_eq^2)*l + (JT(3,3) - JT(1,1))*p_eq*r_eq + JP(3,3)*p_eq*wsum

-gamma*r_eq + kT*kF*(w1_eq^2 - w2_eq^2 + w3_eq^2 - w4_eq^2)

% (f1 + f2 + f3 + f4)*n(3)
% m*g