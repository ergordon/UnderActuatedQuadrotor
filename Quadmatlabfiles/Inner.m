function u = Inner(t,x,tg)
persistent K told
if isempty(K)
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
    B = [0;0;0;86.206896551724140;0];
    Q = diag([20, 20, 0, 0, 0]);
    R = 1;
    An = A(tg(3));
    K = lqr(An,B,Q,R);
end
if isempty(told)
    told = 0;
end
if t-told>.01
    told=t;
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
    B = [0;0;0;86.206896551724140;0];
    Q = diag([20, 20, 0, 0, 0]);
    R = 1;
    An = A(tg(3));
    K = lqr(An,B,Q,R);
end
state = [x(12)/norm(x(10:12));x(11)/norm(x(10:12));x(10);x(11);x(12)];
eqstate = [tg(1);tg(2);0;0;sqrt(1.23e-7*.715*9.81/7.46e-6/1.4e-4)];
ratio = -K*(state-eqstate);
u=zeros(4,1);
u(3)=2*ratio*.17;
u(4)=tg(4);
end