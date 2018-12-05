
syms nx ny nz wx wy wz u real
m = .715;
g = 9.81;
Kdx = .7e-4;
Kdz = 1.4e-4;
kM =  1.23e-7;
kF = 7.46e-6;            
r = sqrt(kM*m*g/(kF*Kdz));
Jx = 4093e-6;
Jz = 7593e-6;
Jpz = 15e-5;
a11 = -Kdx*r/Jx;
a33 = -2*Kdz*r/Jz;
a12 = sqrt(2*m*g)*Jpz/sqrt(kF)/Jx+(Jx-Jz)/Jx*r;

A = @(nz) [0 r 0 -nz 0;
          -r 0 nz 0 0;
          0 0 a11 a12 0;
          0 0 -a12 a11 0;
          0 0 0 0 a33];


