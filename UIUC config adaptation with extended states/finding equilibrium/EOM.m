clc; clear; close all;

syms epsilon nx ny nz   p q r pdot qdot rdot w1 w2 w3 w4 real

% JT = diag([4093e-6 3944e-6 7593e-6]);
% JP = diag([0 0 1.5e-11]);
% JB = JT - 4*JP;


% g = 9.81;
% l = .17;
% m = 0.715;
% kF = 7.46e-6;
% kM = 1.23e-7;
% kT = kM/kF;
% gamma = 2.75e-3;

syms l m gamma kF kT JBx JBy JBz JPx JPy JPz g real
syms JBx JBy JBz JPx JPy JPz real
JB = diag([JBx JBy JBz]);
JP = diag([JPx JPy JPz]);
JT = JB + 4*JP;


n = [nx ny nz]';
wB = [p q r]';
wBdot = [pdot qdot rdot]';


wP1 = [0 0 w1]';
wP2 = [0 0 w2]';
wP3 = [0 0 w3]';
wP4 = [0 0 w4]';

f1 = kF*w1^2;
f2 = kF*w2^2;
f3 = kF*w3^2;
f4 = kF*w4^2;

t1 = kT*f1;
t2 = kT*f2;
t3 = -kT*f3;
t4 = -kT*f4;

td = [0 0 -gamma*r]';

Tres = [l*(f1-f2)+td(1);
        l*(f3-f4)+td(2);
        t1+t2+t3+t4+td(3)];
    
rho = 0.5;
    
eq123 = JB*wBdot + cross(wB, JT*wB + JP*(wP1+wP2+wP3+wP4)) - Tres
eq456 = epsilon*wB - n;
eq7 = 1 - norm(n);
eq8 = m*g - (f1 + f2 + f3 + f4)*nz;
eq9 = w4 ;
eq10 = f2 - f1;
eq11 =  f3/f1 - rho;
eq121314 =  [0;0;0] - wBdot;

