function F = equilibRoot(vars)


rho = 0.5;

epsilon = vars(1);
nx = vars(2);
ny = vars(3);
nz   = vars(4);
p = vars(5);
q = vars(6);
r = vars(7);
pdot = vars(8);
qdot = vars(9);
rdot = vars(10);
w1 = vars(11);
w2 = vars(12);
w3 = vars(13);
w4= vars(14);
    

JT = diag([4093e-6 3944e-6 7593e-6]);
JP = diag([0 0 1.5e-11]);
JB = JT - 4*JP;


g = 9.81;
l = .17;
m = 0.715;
kF = 7.46e-6;
kM = 1.23e-7;
kT = kM/kF;
gamma = 2.75e-3;

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
    

    
F(1:3) = JB*wBdot + cross(wB, JT*wB + JP*(wP1+wP2+wP3+wP4)) - Tres;
F(4:6) = epsilon*wB - n;
F(7) = 1 - norm(n);
F(8) = m*g - (f1 + f2 + f3 + f4)*nz;
F(9) = w4 ;
F(10) = f2 - f1;
F(11) =  f3/f1 - rho;
F(12:14) =  [0;0;0] - wBdot;




    
end