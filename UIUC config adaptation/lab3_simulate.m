function [t, o, theta, v, omega, u, odes] = lab3_simulate()

    %%%%%%%%%%%%%%
    % Parameters
    %%%%%%%%%%%%%%
    eq = getEquilibrium();
    params = getParams(eq);
    gains = getGains(params,eq);
    ii = 0;
    
    % Create variables to keep track of time, state, input, and desired position
    t = [params.t0];
    x = [params.o0; params.theta0; params.v0; params.w0];
    u = [];
    odes = [];

    
    %set the desired positon for each time step here
     %{
     times = params.t0:params.dt:params.t1;
     for i=1:length(times)
         odes = [odes [0;0;-1]];
     end     
    %}
     %set the desired positon for each time step here
     times = params.t0:params.dt:params.t1;
     T = 5;
     radius = 1;
     for i=1:length(times)
         ti = times(i);
         odes = [odes [0;0;-2]];
         %odes = [odes [radius*cos(2*pi*ti/T);radius*sin(2*pi*ti/T);-1]];
     end

     randomT = randi([3 4])
     
    % Iterate over t1/dt sample intervals.
    for i = 1:(params.t1/params.dt)

        
        % Get time and state at start of i'th sample interval
        ti = t(:, i);
        xi = x(:, i);

        %current desired position
        odesi = odes(:,i);

        
        if(t < randomT)
            xe = [0;0;-1;0;0;0;0;0;0;0;0;0];
            ue = [0;0;0;0.715*9.81];
            kF = 7.46e-6;              % aerodynamic force coefficient
            kM = 1.23e-7;              % aerodynamic torque coefficient
            g = 9.81;                 % acceleration of gravity
            m = 0.715;                  % mass
            J = diag([4093e-6, 3944e-6, 7593e-6]);    % moment of inertia matrix in body frame
            l = 0.17;                  % spar length
            sigmamax = 1e3;         % maximum spin rate
            dt = (1/1000);
            K = [-3.834384843641e-14 1.87500770050341 1.20083678667901e-14 -2.16276395182449e-15 1.30571369467938e-14 2.07041831860293 -1.3481323537892e-14 0.89357644863599 2.59310972434929e-14 0.15282805887208 1.84752692644445e-16 -1.90417476791715e-16;-1.85092558905639 -1.48358090939201e-14 9.96004656084597e-14 -5.33797911741503e-15 2.0377399619346 7.65096500245803e-15 -0.880796758304048 1.25991473496734e-15 7.24968623678004e-14 1.408851397387e-16 0.149448159491756 -2.04136342985026e-16;2.44222000191406e-14 -2.22917706532758e-15 5.07914854737093e-14 1.64947334376728 -6.86787340020838e-15 -4.32594072606859e-16 9.27462277081942e-15 -1.16848130868488e-15 3.32997014101363e-14 -9.45550573141918e-17 -1.40999831860081e-16 0.189557251102788;7.53873662611503e-14 -7.82003951475472e-14 -3.06941353921071 -1.87334461479158e-15 -2.96237618206172e-14 -2.05723866592758e-14 2.76673903986577e-14 -3.98647632691682e-14 -2.09954851337643 -3.27662542120601e-16 -6.72794392782925e-16 -3.66968460574694e-16];
                u_desired = -K*(xi-xe) + ue;
                xe = [odesi;xe(4:12)];
                u_desired = -K*(xi-xe) + ue;

                ui = GetBoundedInputs2( u_desired , kF, kM, l, sigmamax);

                %
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                u(:, i) = ui;

                % Get time and state at start of (i+1)'th sample interval
                [tsol, xsol] = ode45(@(t, x) f(t, x, ui, g, m, J), [ti ti+dt], xi);
                t(:, i+1) = tsol(end, :)';
                x(:, i+1) = xsol(end, :)';

                % Get position and orientation
                % store/output state values over time 
                o = xi(1:3, :);
                theta = xi(4:6, :);
                v = xi(7:9, :);
                omega = xi(10:12, :);
    
        else 
            if (ii == 0) 
               d_des = odesi;
               ddot_des = [0;0;0]; %just setting this to zero for now
               theta = xi(4:6);
               d = xi(1:3);
               ddot = xi(7:9);
               a_des = -gains.K_d*(ddot-ddot_des) - gains.K_p*(d-d_des) + eq.a;
               R = Rz(theta(1))*Ry(theta(2))*Rx(theta(3));
               fsum_des = params.m/eq.n(3)*norm(R^(-1)*(a_des-[0;0;params.g]));
               n_des = params.m/(eq.n(3)*fsum_des)*R^(-1)*(a_des-[0;0;params.g]);
               ii = 1;
                eq.s(3) = n_des(1);
                eq.s(4) = n_des(2);
                wB = [xi(10); xi(11); xi(12)];
                n = wB/norm(wB);
                si = [wB(1); wB(2); n(1); n(2)];
                 u_desired = -gains.K_i*(si-eq.s) + eq.u;
                 ui = GetBoundedInputs(u_desired, fsum_des, params);
                [tsol, xsol] = ode45(@(t, x) h(t, x, ui, params), [ti ti+params.dt], xi);
                t(:, i+1) = tsol(end, :)';
                x(:, i+1) = xsol(end, :)';
            else
                
                %run the outter loop at the beginning of every kth step
                if(mod(i-1,params.frequencyRatio)==0)
                   %%%%%%%%%%%%%%%
                   % outter loop
                   %%%%%%%%%%%%%%%


                   %set the desired position and velocity 
                   d_des = odesi;
                   ddot_des = [0;0;0]; %just setting this to zero for now

                   theta = xi(4:6);
                   d = xi(1:3);
                   ddot = xi(7:9);

                   a_des = -gains.K_d*(ddot-ddot_des) - gains.K_p*(d-d_des) + eq.a;
        %            a_des = -gains.K_o*(d-d_des) + a_eq;

                   %setpoints were derived from (45) and the fact that n_des is a
                   %unit vector
                   R = Rz(theta(1))*Ry(theta(2))*Rx(theta(3));

                   %inner loop set points!!!
                   %fsum gets passed into the getbounded inputs function to enforce
                   %the total force condition
                   fsum_des = params.m/eq.n(3)*norm(R^(-1)*(a_des-[0;0;params.g]));

                   %n_des is set as the equilibrium for the inner loop reduced
                   %attitude state. 
                   n_des = params.m/(eq.n(3)*fsum_des)*R^(-1)*(a_des-[0;0;params.g]);

                end
                %%%%%%%%%%%%%%%%%
                % inner loop
                %%%%%%%%%%%%%%%%%
                %set equilibrium nx and ny so the inner loop input can try to align
                %the body n with the n_des set in the outter loop
                eq.s(3) = n_des(1);
                eq.s(4) = n_des(2);

                %current angular velocity
                wB = [xi(10); xi(11); xi(12)];
                %current primary axis of rotation
                n = wB/norm(wB);

                %current reduced attitude state
                si = [wB(1); wB(2); n(1); n(2)];

                %%%%%%%%%%%%%%%%%%%%%%%%%%
                %not extended motor states
                %%%%%%%%%%%%%%%%%%%%%%%%%%
                 u_desired = -gains.K_i*(si-eq.s) + eq.u;
                 ui = GetBoundedInputs(u_desired, fsum_des, params);

                % Get time and state at start of (i+1)'th sample interval
                [tsol, xsol] = ode45(@(t, x) h(t, x, ui, params), [ti ti+params.dt], xi);

                t(:, i+1) = tsol(end, :)';
                x(:, i+1) = xsol(end, :)';
        end


        % store/output state values over time 
        o = x(1:3, :);
        theta = x(4:6, :);
        v = x(7:9, :);
        omega = x(10:12, :);
        end
    end
end

function xdot = h(t, x, u, params)

    
    o = x(1:3);
    t = x(4:6);
    v = x(7:9);
    p = x(10);
    q = x(11);
    r = x(12);
    
    %ZYX
    R = Rz(t(1))*Ry(t(2))*Rx(t(3));
    N = [Rx(t(3))'*Ry(t(2))'*[0;0;1] Rx(t(3))'*[0;1;0] [1;0;0]]^(-1);

    odot = v;
    tdot = N*[p;q;r];
    
    vdot = 1/params.m*([0;0;params.m*params.g] + R*[0;0;-params.kF*(u(1)^2+u(2)^2+u(3)^2+u(4)^2)]);
   
    kF = params.kF;
    kT = params.kT;
    JBx = params.JB(1,1);
    JBy = params.JB(2,2);
    JBz = params.JB(3,3);
    JPx = params.JP(1,1);
    JPy = params.JP(2,2);
    JPz = params.JP(3,3);
    w1 = u(1);
    w2 = u(2);
    w3 = u(3);
    w4 = u(4);
    l = params.el;
    gamma = params.gamma;
    
    pdot = -1/JBx*(q*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) - l*(kF*w1^2 - kF*w2^2) - q*r*(JBy + 4*JPy));
    qdot = -1/JBy*(-p*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) - l*(kF*w3^2 - kF*w4^2) + p*r*(JBx + 4*JPx));
    rdot = -1/JBz*(- kF*kT*w1^2 - kF*kT*w2^2 + kF*kT*w3^2 + kF*kT*w4^2 + gamma*r - p*q*(JBx + 4*JPx) + p*q*(JBy + 4*JPy));
    
    
%     pdot = 1/params.JB(1,1)*(params.kF*(u(1)^2-u(2)^2)*params.el - (params.JT(2,2) - params.JT(3,3))*q*r - params.JP(3,3)*q*(u(1)+u(2)+u(3)+u(4)));
%     qdot = 1/params.JB(2,2)*(params.kF*(u(3)^2-u(4)^2)*params.el + (params.JT(3,3) - params.JT(1,1))*p*r + params.JP(3,3)*p*(u(1)+u(2)+u(3)+u(4)));
%     rdot = 1/params.JB(3,3)*(-params.gamma*r + p*q*(params.JT(1,1)-params.JT(2,2)) + params.kT*params.kF*(u(1)^2+u(2)^2-u(3)^2-u(4)^2));
    wdot = [pdot;qdot;rdot];
    
    xdot = [odot;tdot;vdot;wdot];

end


function [u, sigma] = GetBoundedInputs(u_desired, fsum_des, params)

	%u comes in as [f3-f1;f2] (the equilibriums offsets were already removed before being passed in)   
    
    %these are derived from equations (29)(30)(31)
    f3 = u_desired(2);
    f2 = 1/2*(fsum_des + u_desired(1) - f3);
    f1 = fsum_des - f3 - f2;
    

    if(f1 < 0)
        f1 = 0;
    end
    if(f2 < 0)
        f2 = 0;
    end
    if(f3 < 0)
        f3 = 0;
    end


    s1 = sqrt(f1/params.kF);
    if(s1 > params.sigmamax)
        s1 = params.sigmamax;
    end
    if(s1 < 0)
        s1 = 0;
    end 
    
    s2 = sqrt(f2/params.kF);
    if(s2 > params.sigmamax)
        s2 = params.sigmamax;
    end
    if(s2 < 0)
        s2 = 0;
    end 
    
    s3 = sqrt(f3/params.kF);
    if(s3 > params.sigmamax)
        s3 = params.sigmamax;
    end
    if(s3 < 0)
        s3 = 0;
    end

    
    u = [s1; s2; s3; 0];
    
end

function xdot = f(t, x, u, g, m, J)

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


function [u, sigma] = GetBoundedInputs2(u_desired, kF, kM, l, sigmamax)

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

    u = u_desired;
    W = [l*kF -l*kF 0 0; 0 0 l*kF -l*kF; kM kM -kM -kM; kF kF kF kF];
    sigma_d = W^-1*u;
    sigma_d(sigma_d > sigmamax^2) =  sigmamax^2;
    sigma_d(sigma_d < 0) =  0;
    sigma = sigma_d;
    u = W*sigma;
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
