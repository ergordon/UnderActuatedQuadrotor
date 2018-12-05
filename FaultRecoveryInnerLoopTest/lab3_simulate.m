function [t, o, theta, v, omega, odes] = lab3_simulate()

    %%%%%%%%%%%%%%
    % Parameters
    %%%%%%%%%%%%%%
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
    
    %load the stored equilibrium values
    load('equilib.mat')
    
    %initial conditions
     o0 = [0; 0; -1];
    theta0 = [0; 0; 0];
    v0 = [0; 0; 0];
    w0 = [p_eq;q_eq;r_eq];
    
    % time
    dt = (1/50);
    t0 = 0;
    t1 = 6;
   
    %equilibrium reduced attitude and input
    se = [p_eq;q_eq;nx_eq;ny_eq]; %[p;q;nx;ny]
    ue = [f1_eq-f3_eq;f2_eq]; %[f1-f3;f2]
   
   
   %%%%%%%%%%%%%%%%%%%%
   %controller gains
   %%%%%%%%%%%%%%%%%%%%
   
   %this is used for the inner loop
   K = [-0.0000    1.0639   -3.6615   -2.5678;
        1.0639   -0.0000   -2.5678    3.6615];

   %these are used for the outter loop
   %damping ratio
   sigma = 0.7;
   %natural frequency
   omega_n = 1;

    
    % Create variables to keep track of time, state, input, and desired position
    t = [t0];
    x = [o0; theta0; v0; w0];
    u = [];
    odes = [];

    
    %set the desired positon for each time step here
     times = t0:dt:t1;
     for i=1:length(times)
         odes = [odes [0;0;-1]];
     end     

    % Iterate over t1/dt sample intervals.
    for i = 1:(t1/dt)

        
        % Get time and state at start of i'th sample interval
        ti = t(:, i);
        xi = x(:, i);

        %current desired position
        odesi = odes(:,i);

        
        %run the outter loop at the beginning of every kth step
        k = 10;
        if(mod(i-1,k)==0)
           %%%%%%%%%%%%%%%
           % outter loop
           %%%%%%%%%%%%%%%
           
           
           %equilibrium acceleration is zero
           a_eq = [0;0;0];
           %set the desired position and velocity 
           d_des = odesi;
           ddot_des = [0;0;0]; %just setting this to zero for now
           
           theta = xi(4:6);
           d = xi(1:3);
           ddot = xi(7:9);
           
           %u = -K_d*deltaxdot - Kd*deltax
           a_des = -2*sigma*omega_n*(ddot-ddot_des) - omega_n^2*(d-d_des) + a_eq;
           
           %these are wrong i think. I thnk the n from one paper is in the
           %inertial frame and the n in another paper is in the body frame
           %i could be wrong though
%            %innner loop set point!!!!
%            n_des = (a_des-[0;0;-g])/norm(a_des-[0;0;-g]);
%            %inner loop set point!!!
%            fsum_des = m*norm(a_des - [0;0;-g])/nz_eq;
          

           %setpoints were derived from (45) and the fact that n_des is a
           %unit vector
           R = Rz(theta(1))*Ry(theta(2))*Rx(theta(3));
           
           %inner loop set points!!!
           %fsum gets passed into the getbounded inputs function to enforce
           %the total force condition
           fsum_des = m/nz_eq*norm(R^(-1)*(a_des-[0;0;-g]));
           
           %n_des is set as the equilibrium for the inner loop reduced
           %attitude state. 
           n_des = m/(nz_eq*fsum_des)*R^(-1)*(a_des-[0;0;-g]);
           
           
        end
        
        
        %%%%%%%%%%%%%%%%%
        % inner loop
        %%%%%%%%%%%%%%%%%
        
         %i might need to change the equilibrium forces values??????? 
         %dont think so, this is the only way to enforce rho, which is a 
         %tuning factor.
%         se = [p_eq;q_eq;nx_eq;ny_eq]; %[p;q;nx;ny]
%         ue = [f1_eq-f3_eq;f2_eq]; %[f1-f3;f2] 
        
        %set equilibrium nx and ny so the inner loop input can try to align
        %the body n with the n_des set in the outter loop
        se(3) = n_des(1);
        se(4) = n_des(2);
        
        
        %current angular velocity
        wB = [xi(10); xi(11); xi(12)];
        %current primary axis of rotation
        n = wB/norm(wB);
        
        %current reduced attitude state
        si = [wB(1); wB(2); n(1); n(2)];

        u_desired = -K*(si-se) + ue;
        ui = GetBoundedInputs(u_desired, fsum_des, kF, sigmamax);
        
        u(:, i) = ui;

        % Get time and state at start of (i+1)'th sample interval
        [tsol, xsol] = ode45(@(t, x) h(t, x, ui, g, m, JB, JP, gamma, kF, kT, l), [ti ti+dt], xi);
   
        t(:, i+1) = tsol(end, :)';
        x(:, i+1) = xsol(end, :)';

    end

    
    % store/output state values over time 
    o = x(1:3, :);
    theta = x(4:6, :);
    v = x(7:9, :);
    omega = x(10:12, :);

end

function xdot = h(t, x, u, g, m, JB, JP, gamma, kF, kT, l)

    
    JT = JB + 4*JP;
    
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
    
    vdot = 1/m*([0;0;-m*g] + R*[0;0;kF*(u(1)^2+u(2)^2+u(3)^2+u(4)^2)]);
   
    pdot = 1/JB(1,1)*(kF*(u(2)^2-u(4)^2)*l - (JT(3,3) - JT(1,1))*q*r - JP(3,3)*q*(u(1)+u(2)+u(3)+u(4)));
    qdot = 1/JB(1,1)*(kF*(u(3)^2-u(1)^2)*l + (JT(3,3) - JT(1,1))*p*r + JP(3,3)*p*(u(1)+u(2)+u(3)+u(4)));
    rdot = 1/JB(3,3)*(-gamma*r + kT*kF*(u(1)^2-u(2)^2+u(3)^2-u(4)^2));
    wdot = [pdot;qdot;rdot];
    
    xdot = [odot;tdot;vdot;wdot];

end


function [u, sigma] = GetBoundedInputs(u_desired, fsum_des, kF, sigmamax)

	%u comes in as [f3-f1;f2] (the equilibriums offsets were already removed before being passed in)   
    
    %these are derived from equations (29)(30)(31)
    f2 = u_desired(2);
    f3 = 1/2*(fsum_des + u_desired(1) - f2);
    f1 = fsum_des - f2 - f3;
    

    %should i do this before calculating the f values or just floor the f
    %values after calculaitng them?
%     if(u_desired(1) < 0)
%         u_desired(1) = 0;
%     end
%     if(u_desired(2) < 0)
%         u_desired(2) = 0;
%     end
    

    if(f1 < 0)
        f1 = 0;
    end
    if(f2 < 0)
        f2 = 0;
    end
    if(f3 < 0)
        f3 = 0;
    end


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

    
    u = [s1; s2; s3; 0];
    
end
