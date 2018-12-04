clc; clear; close all;

syms o1 o2 o3 th1 th2 th3 v1 v2 v3 w1 w2 w3 u1 u2 u3 u4 g m Jxx Jyy Jzz real;

xs = [o1 o2 o3 th1 th2 th3 v1 v2 v3 w1 w2 w3]';
us = [u1 u2 u3 u4]';
Js = diag([Jxx Jyy Jzz]);

xe = [0 0 0 0 0 0 0 0 0 0 0 0]';
ue = [0 0 0 m*g]';

fs = h(0,xs,us,g,m,Js);

A = subs(jacobian(fs,xs),[xs;us],[xe;ue]);
B = subs(jacobian(fs,us),[xs;us],[xe;ue]);

xdot = A*xs+B*us








function xdot = h(t, x, u, g, m, J)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Compute xdot given x, u, and other parameters (see HW2.2)

    
    o = x(1:3);
    t = x(4:6);
    v = x(7:9);
    w = x(10:12);
    
    what = [0 -w(3) w(2);
            w(3) 0 -w(1);
            -w(2) w(1) 0];
    %ZYX
    R = Rz(t(1))*Ry(t(2))*Rx(t(3));
    
    N = [Rx(t(3))'*Ry(t(2))'*[0;0;1] Rx(t(3))'*[0;1;0] [1;0;0]]^(-1);
    
    odot = v;
    tdot = N*w;
    vdot = 1/m*([0;0;m*g] + R*[0;0;-u(4)]);
    wdot = J^(-1)*(u(1:3) - what*J*w);
    
    xdot = [odot;tdot;vdot;wdot];

    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end