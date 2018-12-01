function [t, o, theta, omega, odes] = lab3_simulate()

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Change parameter values and sample time to match your quadrotor (see
    %     your results from Labs #1 and #2)
    %   - Change initial time, final time, and initial conditions as you like
    %

    % Parameters
    JT = diag([3.2e-3 3.2e-3 5.5e-3]);
    JP = diag([0 0 1.5e-5]);
    JB = JT - 4.*JP;

    g = 9.81;
    l = .17;
    m = 0.5;
    kF = 6.41e-6;
    kT = 1.69e-2;
    gamma = 2.75e-3;
    

    sigmamax = 1000;
    

    %initial conditions
     o0 = [0; 0; -1];
    theta0 = [0; 0; 0];
    v0 = [0; 0; 0];
    w0 = [0;5.69;18.89];
    
    % time
    dt = (1/50);
    t0 = 0;
    t1 = 6;

    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   Patching in my controller
    %
    %
%         load('equilib.mat')
%     
%         se = [p q nx ny]';
%         ue = [w1 w2 w3 w4]';
%     
%         K = [-0.0267   -0.9105   -0.0367    0.8731;
%              0.6702   -0.0075   -0.6651    0.0696;
%              -0.0472    0.9114    0.1101   -0.8807;
%              -0.0370    0.0004    0.0367   -0.0038];


           se = [0;5.69;0;.289]; %[p;q;nx;ny]
           ue = [0;1.02]; %[f1-f3;f2]
           
           K = [-0.0000    1.0640   -3.6615   -2.5677;
                1.0640   -0.0000   -2.5677    3.6615];


    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    % Create variables to keep track of time, state, input, and desired position
    t = [t0];
    x = [o0; theta0; v0; w0];
    u = [];
    odes = [];

     times = t0:dt:t1;
     for i=1:length(times)
         odes = [odes [0;0;-1]];
     end     

    % Iterate over t1/dt sample intervals.
    for i = 1:(t1/dt)

        % Get time and state at start of i'th sample interval
        ti = t(:, i);
        xi = x(:, i);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %   MODIFY
        %
        %   - Get desired position at start of i'th sample interval
        %
        %     (You may also need to redefine your equilibrium point!)
        %

        odesi = odes(:,i);

        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         odes(:, i) = odesi;

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %
        %   MODIFY
        %
        %   - Get input that will be applied throughout the i'th sample
        %     interval (in other words, implement your control policy)
        %   - Don't forget to make sure that this input can be realized by
        %     spin rates that are between 0 and sigmamax
        %
        
%         xe = [odesi;xe(4:12)];
%         u_desired = -K*(xi-xe) + ue;
%         ui = GetBoundedInputs( u_desired , kF, kM, l, sigmamax);

        wB = [xi(10); xi(11); xi(12)];
        
%         n = JT*wB/norm(JT*wB);
        n = wB/norm(wB);
        
        si = [wB(1); wB(2); n(1); n(2)];
        
%         si = [xi(5) xi(6) nx ny]';

        
        u_desired = -K*(si-se) + ue;
        ui = GetBoundedInputs2( u_desired , kF, kT, l, sigmamax, n);
        
        
%         ui = -K*(si-se) + ue
%         ui = [0 0 0 0]';

        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        u(:, i) = ui;

        % Get time and state at start of (i+1)'th sample interval
        [tsol, xsol] = ode45(@(t, x) h(t, x, ui, g, m, JB, JP, gamma, kF, kT, l), [ti ti+dt], xi);
        t(:, i+1) = tsol(end, :)';
        x(:, i+1) = xsol(end, :)';
%         xsol(10:12)
    end

    % Get position and orientation
    o = x(1:3, :);
    theta = x(4:6, :);
    omega = x(10:12, :);

end

function xdot = h(t, x, u, g, m, JB, JP, gamma, kF, kT, l)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Compute xdot given x, u, and other parameters (see HW2.2)
    
    JT = JB + 4*JP;
    
    o = x(1:3);
    t = x(4:6);
    v = x(7:9);
    p = x(10);
    q = x(11);
    r = x(12);
    
    
    
    %XYZ
    R = Rx(t(1))*Ry(t(2))*Rz(t(3));
    
    
    N = [Rz(t(3))'*Ry(t(2))'*[1;0;0] Rz(t(3))'*[0;1;0] [0;0;1]]^(-1);
    
    odot = v;
    tdot = N*[p;q;r];
    
    
%      neq = [0;.289;.958];
%      vdot = 1/m*([0;0;-m*g] + R*kF*(u(1)^2+u(2)^2+u(3)^2+u(4)^2)*neq(3)*neq);
    
    vdot = 1/m*([0;0;-m*g] + R*[0;0;kF*(u(1)^2+u(2)^2+u(3)^2+u(4)^2)]);
   
    pdot = 1/JB(1,1)*(kF*(u(2)^2-u(4)^2)*l - (JT(3,3) - JT(1,1))*q*r - JP(3,3)*q*(u(1)+u(2)+u(3)+u(4)));
    qdot = 1/JB(1,1)*(kF*(u(3)^2-u(1)^2)*l + (JT(3,3) - JT(1,1))*p*r + JP(3,3)*p*(u(1)+u(2)+u(3)+u(4)));
    rdot = 1/JB(3,3)*(-gamma*r + kT*kF*(u(1)^2-u(2)^2+u(3)^2-u(4)^2));
    wdot = [pdot;qdot;rdot];
    
    xdot = [odot;tdot;vdot;wdot];

    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end

function [u, sigma] = GetBoundedInputs(u_desired, kF, kM, l, sigmamax)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Compute an input u (4x1) that is close to the input u_desired but
    %     that can be realized by spin rates that are between 0 and sigmamax
    %   - Compute the spin rates sigma (4x1) - these are not used by the rest
    %     of the simulation code, but will be necessary in your onboard C code,
    %     so it is useful to make sure you know how to compute them here
    %
    %     (See HW2.1.2)

%     u = u_desired;
%     W = [l*kF -l*kF 0 0;
%         0 0 l*kF -l*kF;
%         kM kM -kM -kM;
%         kF kF kF kF];
%     sigma_d = W^-1*u;
%     sigma_d(sigma_d > sigmamax^2) =  sigmamax^2;
%     sigma_d(sigma_d < 0) =  0;
%     sigma = sigma_d;
%     u = W*sigma;    


    W = [l*kF -l*kF 0 0;
        0 0 l*kF -l*kF;
        kM kM -kM -kM;
        kF kF kF kF];
    
%     sigma_d = (W'*W)^(-1)*W'*u;
    u_desired(u_desired > sigmamax) =  sigmamax;
    u_desired(u_desired < 0) =  0;
    
    for i=1:4
       u_desired(i) = u_desired(i)^2; 
    end
    
    u = W*u_desired;
    
    
    
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end


function [u, sigma] = GetBoundedInputs2(u_desired, kF, kM, l, sigmamax, n)

   
	%u comes in as [f3-f1;f2]
    

    if(u_desired(1) < 0)
        u_desired(1) = 0;
    end
    if(u_desired(2) < 0)
        u_desired(2) = 0;
    end
    
    syms f1 f2 f3 f4 real
    
    feqsum = 5.12;
%     eq1 = u_desired(1) == f3 - f1;
%     eq2 = f2 == u_desired(2);
%     eq3 = f1 + f2 + f3 == feqsum;
% %     eq3 = f1 + f2 + f3 == u_desired(1)+u_desired(2);
%     
%     sol = solve([eq1 eq2 eq3],[f1 f2 f3]);
%     f1 = double(sol.f1);
%     f2 = double(sol.f2);
%     f3 = double(sol.f3);
    
    f2 = u_desired(2);
    f3 = 1/2*(feqsum + u_desired(1) - f2);
    f1 = feqsum - f2 - f3;
    
    s1 = sqrt(f1/kF);
    if(s1 > sigmamax)
        s1 = sigmamax;
    end
    if(s1 < 0)
        s1 = 0;
    end 
    
    s2 = sqrt(f2/kF);
    if(s2 > sigmamax)
        s2 = sigmamax;
    end
    if(s2 < 0)
        s2 = 0;
    end 
    
    s3 = sqrt(f3/kF);
    if(s3 > sigmamax)
        s3 = sigmamax;
    end
    if(s3 < 0)
        s3 = 0;
    end
    
%     totaleq = sqrt(5.12/kF);

%     leftover = 5.12 - kF*s2^2;
%     s1and3 = sqrt(.5*leftover/kF);
    
%     totalf = kF*(s1and3^2 + s2^2 + s1and3^2)
%     totalf = (s1and3^2 + s2^2 + s1and3^2)
    
    
    u = [s1; s2; s3; 0];

end
