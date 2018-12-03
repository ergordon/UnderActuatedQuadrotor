function out=GetBoundedInputs(f,u,t,x)
l = 0.17;                  % spar length
kF = 7.46e-6;              % aerodynamic force coefficient
kM = 1.23e-7;              % aerodynamic torque coefficient
sigmamax = 1e3;            % maximum spin rate

u = u(t,x);
W = [l*kF -l*kF 0 0; 0 0 l*kF -l*kF; kM kM -kM -kM; kF kF kF kF];
sigma_d = W^-1*u;
sigma_d(sigma_d > sigmamax^2) =  sigmamax^2;
sigma_d(sigma_d < 0) =  0;
%% 
sigma = sigma_d;
u = W*sigma;

out=f(t,x,u);
end