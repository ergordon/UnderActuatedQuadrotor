wn = 1;
epslion = .7;
A = [0 1;-wn^2 -2*epslion*wn];

syms c1 c2 c3 s1 s2 s3 nx ny nz m f g
ndes=[nx; ny; nz]
R = [c1*c2 c1*s2*s3-c3*s1 s1*s3+c1*c3*s2;c2*s1 c1*c3+s1*s2*s3 c3*s1*s2-c1*s3;-s2 c2*s3 c2*c3]
R*(ndes*nz*f/m)+[0;0;g]
jacobian(ans,[nx ny nz, f])
%%
% B = [(c1*c2*f*nz)/m, -(f*nz*(c3*s1 - c1*s2*s3))/m, (2*f*nz*(s1*s3 + c1*c3*s2))/m - (f*ny*(c3*s1 - c1*s2*s3))/m + (c1*c2*f*nx)/m;
%     (c2*f*nz*s1)/m, (f*nz*(c1*c3 + s1*s2*s3))/m, (f*ny*(c1*c3 + s1*s2*s3))/m - (2*f*nz*(c1*s3 - c3*s1*s2))/m + (c2*f*nx*s1)/m;
%     -(f*nz*s2)/m, (c2*f*nz*s3)/m, (2*c2*c3*f*nz)/m - (f*nx*s2)/m + (c2*f*ny*s3)/m];

B2 = [ (c1*c2*f)/m, -(f*(c3*s1 - c1*s2*s3))/m,  (f*(s1*s3 + c1*c3*s2))/m;
    (c2*f*s1)/m,  (f*(c1*c3 + s1*s2*s3))/m, -(f*(c1*s3 - c3*s1*s2))/m;
    -(f*s2)/m,               (c2*f*s3)/m,               (c2*c3*f)/m];

B3 = [ (c1*c2*f)/m, -(f*(c3*s1 - c1*s2*s3))/m;
    (c2*f*s1)/m,  (f*(c1*c3 + s1*s2*s3))/m;
    -(f*s2)/m,               (c2*f*s3)/m];

B4 = [ (c1*c2*f*(- nx^2 - ny^2 + 1)^(1/2))/m - (c1*c2*f*nx^2)/(m*(- nx^2 - ny^2 + 1)^(1/2)) + (f*nx*ny*(c3*s1 - c1*s2*s3))/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (f*nx*nz*(s1*s3 + c1*c3*s2))/(m*(- nx^2 - ny^2 + 1)^(1/2)), (f*ny^2*(c3*s1 - c1*s2*s3))/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (f*(c3*s1 - c1*s2*s3)*(- nx^2 - ny^2 + 1)^(1/2))/m - (f*ny*nz*(s1*s3 + c1*c3*s2))/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (c1*c2*f*nx*ny)/(m*(- nx^2 - ny^2 + 1)^(1/2));
    (c2*f*s1*(- nx^2 - ny^2 + 1)^(1/2))/m - (c2*f*nx^2*s1)/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (f*nx*ny*(c1*c3 + s1*s2*s3))/(m*(- nx^2 - ny^2 + 1)^(1/2)) + (f*nx*nz*(c1*s3 - c3*s1*s2))/(m*(- nx^2 - ny^2 + 1)^(1/2)), (f*(c1*c3 + s1*s2*s3)*(- nx^2 - ny^2 + 1)^(1/2))/m - (f*ny^2*(c1*c3 + s1*s2*s3))/(m*(- nx^2 - ny^2 + 1)^(1/2)) + (f*ny*nz*(c1*s3 - c3*s1*s2))/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (c2*f*nx*ny*s1)/(m*(- nx^2 - ny^2 + 1)^(1/2));
    (f*nx^2*s2)/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (f*s2*(- nx^2 - ny^2 + 1)^(1/2))/m - (c2*c3*f*nx*nz)/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (c2*f*nx*ny*s3)/(m*(- nx^2 - ny^2 + 1)^(1/2)),                                           (c2*f*s3*(- nx^2 - ny^2 + 1)^(1/2))/m - (c2*f*ny^2*s3)/(m*(- nx^2 - ny^2 + 1)^(1/2)) + (f*nx*ny*s2)/(m*(- nx^2 - ny^2 + 1)^(1/2)) - (c2*c3*f*ny*nz)/(m*(- nx^2 - ny^2 + 1)^(1/2))];

Q = diag([100 100]);
R = diag([1 1]);
th1 = 0;
th2 = 0;
th3 = 0;
Beq = [cos(th1),cos(th2),cos(th3),sin(th1),sin(th2),sin(th3),0,0,1,0.715,9.81*0.715,9.81];
B=double(subs(B3,[c1,c2,c3,s1,s2,s3,nx,ny,nz,m,f,g],Beq))
[~,~,K3]=dare(A,B(1:2,:),Q,R)
B=double(subs(B2,[c1,c2,c3,s1,s2,s3,nx,ny,nz,m,f,g],Beq))
[~,~,K2]=dare(A,B(1:2,1:2),Q,R)
B=double(subs(B4,[c1,c2,c3,s1,s2,s3,nx,ny,nz,m,f,g],Beq))
[~,~,K4]=dare(A,B(1:2,1:2),Q,R)

%%
Jp = 0.016 * 9.9865 * 10 ^ (- 6 ); 
Jtz = 7593e-6;
Jtx = 4093e-6;
Jbx = 4093e-6- 4*Jp;
r = 1


