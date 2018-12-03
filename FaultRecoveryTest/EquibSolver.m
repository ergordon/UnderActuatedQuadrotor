clc; clear; close all;


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



syms nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4 w1dot w2dot w3dot w4dot real
% assume(w1 > 0)
% assume(w2 > 0)
% assume(w3 > 0)
% assume(w4 > 0)




wB = [p q r]';
wBdot = [pdot qdot rdot]';
% wBdot = [0 0 0]';

wP1dot = [0 0 w1dot]';
wP2dot = [0 0 w2dot]';
wP3dot = [0 0 w3dot]';
wP4dot = [0 0 w4dot]';

wP1 = [0 0 w1]';
wP2 = [0 0 w2]';
wP3 = [0 0 w3]';
wP4 = [0 0 w4]';




f1 = kF*w1^2;
f2 = kF*w2^2;
f3 = kF*w3^2;
f4 = kF*w4^2;



t1 = (-1)^(1+1)*kT*f1;
t2 = (-1)^(2+1)*kT*f2;
t3 = (-1)^(3+1)*kT*f3;
t4 = (-1)^(4+1)*kT*f4;

td = [0 0 -gamma*r]';

T12 = JB*wBdot;% + JP*(wP1dot+wP2dot+wP3dot+wP4dot);
T3 = cross(wB,(JB+4*JP)*wB + JP*(wP1+wP2+wP3+wP4));
Tres = [(f2-f4)*l+td(1);
        (f3-f1)*l+td(2);
         t1+t2+t3+t4+td(3)];

eq123 = T12 + T3 == Tres;

n = [nx ny nz]';

eq456 = n == epsilon*wB;
eq7 = norm(epsilon*wB) == 1;
eq8 = (f1+f2+f3+f4)*nz == m*g;

%one prop failed
rho = 0.6;

eq9 = f4 == 0;
eq10 = f1 == f3;

eq121314 = wBdot == [0 0 0]';

rhospace = linspace(0,3,10);
f1space = [];
f2space = [];
f3space = [];
f4space = [];
%%
for i=1:length(rhospace)
   rho = rhospace(i);
   eq11 = f2/f1 == rho;

   sol = solve([eq123 eq456 eq7 eq8 eq9 eq10 eq11 eq121314], [nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4]);
%    nx = double(sol.nx)
%     ny = double(sol.ny)
%     nz = double(sol.nz)
%     epsilon = double(sol.epsilon)
%     p = double(sol.p)
%     q = double(sol.q)
%     r = double(sol.r)
%     pdot = double(sol.pdot)
%     qdot = double(sol.qdot)
%     rdot = double(sol.rdot)
    f1space = [f1space kF*double(sol.w1)^2];
    f2space = [f2space kF*double(sol.w2)^2];
    f3space = [f3space kF*double(sol.w3)^2];
    f4space = [f4space kF*double(sol.w4)^2]; 
    
end
%%
figure()
plot(rhospace(1:6),f1space,rhospace(1:6),f2space,rhospace(1:6),f3space,rhospace(1:6),f4space)

% sol = solve([eq123 eq456 eq7 eq8 eq9 eq10 eq11], [nx ny nz epsilon p q r w1 w2 w3 w4]);

%%

nx = double(sol.nx)
ny = double(sol.ny)
nz = double(sol.nz)
epsilon = double(sol.epsilon)
p = double(sol.p)
q = double(sol.q)
r = double(sol.r)
pdot = double(sol.pdot)
qdot = double(sol.qdot)
rdot = double(sol.rdot)
w1 = double(sol.w1)
w2 = double(sol.w2)
w3 = double(sol.w3)
w4 = double(sol.w4)

save equilib.mat nx ny nz epsilon p q r pdot qdot rdot w1 w2 w3 w4
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
rho = 0.6;


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
% load('equilib.mat');


JT = diag([3.2e-3 3.2e-3 5.5e-3]);
JP = diag([0 0 1.5e-5]);
JB = JT - 4.*JP;

g = 9.81;
l = .17;
m = 0.5;
kF = 6.41e-6;
kT = 1.69e-2;
gamma = 2.75e-3;

f1eq = 2.05
f3eq = 2.05
f2eq = 1.02
wBeq = [0;5.69;18.89]; %p,q,r
neq = [0;.289;.958];
rho = 0.5;

w1eq = sqrt(f1eq/kF);
w2eq = sqrt(f2eq/kF);
w3eq = sqrt(f3eq/kF);
w4eq = 0;


n = wBeq/norm(wBeq)
% n = JT*wBeq/norm(JT*wBeq)


aeq = ( JT(1,1)-JT(3,3) )/JB(1,1)*wBeq(3) - JP(3,3)/JB(1,1)*(w1eq+w2eq+w3eq+w4eq);

A = [0 aeq 0 0;
     -aeq 0 0 0;
     0 -neq(3) 0 wBeq(3);
     neq(3) 0 -wBeq(3) 0];
B = l/JB(1,1).*[0 1;
                1 0;
                0 0;
                0 0];

Q = diag([1 1 20 20]);
R = diag([1 1]);

K = lqr(A, B, Q, R)
