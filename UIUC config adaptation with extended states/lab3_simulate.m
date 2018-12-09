function [t, o, theta, v, omega, u, odes] = lab3_simulate()

    %%%%%%%%%%%%%%
    % Parameters
    %%%%%%%%%%%%%%
    eq = getEquilibrium();
    params = getParams(eq);
    gains = getGains(params,eq);

    
    % Create variables to keep track of time, state, input, and desired position
    t = [params.t0];
    x = params.x0;
    u = [];
    odes = [];

    
    %set the desired positon for each time step here
     times = params.t0:params.dt:params.t1;
     for i=1:length(times)
         odes = [odes [0;0;-1]];
     end     

    % Iterate over t1/dt sample intervals.
    for i = 1:(params.t1/params.dt)

        
        % Get time and state at start of i'th sample interval
        ti = t(:, i);
        xi = x(:, i);

        %current desired position
        odesi = odes(:,i);

        
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
         
           %use different gains for x,y,z
%            a_des(1) = -gains.K_dx*(ddot(1)-ddot_des(1)) - gains.K_px*(d(1)-d_des(1)) + eq.a(1);
%            a_des(2) = -gains.K_dy*(ddot(2)-ddot_des(2)) - gains.K_py*(d(2)-d_des(2)) + eq.a(2);
%            a_des(3) = -gains.K_dz*(ddot(3)-ddot_des(3)) - gains.K_pz*(d(3)-d_des(3)) + eq.a(3);

           
           
           %use a PID controller that was converted to a pid controller
           a_des = -gains.K_o*(d-d_des) + eq.a;

           %setpoints were derived from (45) and the fact that n_des is a
           %unit vector
           R = Rz(theta(1))*Ry(theta(2))*Rx(theta(3));
           
           %inner loop set points!!!
           %fsum gets passed into the getbounded inputs function to enforce
           %the total force condition
           fsum_des = params.m/eq.n(3)*norm((R')*(a_des-[0;0;params.g]));
           
           
           %n_des is set as the equilibrium for the inner loop reduced
           %attitude state. 
%            n_des = params.m/(eq.n(3)*fsum_des)*R^(-1)*(a_des-[0;0;params.g]);
           n_des = params.m/(eq.n(3)*fsum_des)*(R')*(a_des-[0;0;params.g]);
           
           
           
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
        if(norm(wB) == 0) %prevent division by zero if not rotating
            n = -[0;0;1];
        else
            n = wB/norm(wB);    
        end
        
        %current reduced attitude state
        si = [wB(1); wB(2); n(1); n(2)];

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %not extended motor states
        %%%%%%%%%%%%%%%%%%%%%%%%%%
        u_desired = -gains.K_i*(si-eq.s) + eq.u;
        ui = GetBoundedInputs(u_desired, fsum_des, params);
%         eq.s_ext(5) = params.kF*(ui(2)^2 - ui(1)^2);
%         eq.s_ext(6) = params.kF*ui(3)^2;

        
        
       %%%%%%%%%%%%%%%%%%%%%%%%%%
        %extended motor states
        %%%%%%%%%%%%%%%%%%%%%%%%%%
                       
        %extended state
        %get current spin rates and put in terms of u (f)
%         w1i = xi(13);
%         w2i = xi(14);
%         w3i = xi(15);
%         
%         si_ext = [wB(1); wB(2); n(1); n(2); params.kF*(w2i^2-w1i^2); params.kF*w3i^2];
% 
%         u_desired = -gains.K_i_ext*(si_ext - eq.s_ext) + eq.u;
%         ui = GetBoundedInputs(u_desired, fsum_des, params);

        u(:, i) = ui;
        
       
        
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

%this one has inputs as a state variable
function xdot = h(t, x, u, params)



    o = x(1:3);
    t = x(4:6);
    v = x(7:9);
    p = x(10);
    q = x(11);
    r = x(12);
    motorRates = x(13:16);


    kF = params.kF;
    kT = params.kT;
    JBx = params.JB(1,1);
    JBy = params.JB(2,2);
    JBz = params.JB(3,3);
    JPx = params.JP(1,1);
    JPy = params.JP(2,2);
    JPz = params.JP(3,3);
    
    w1 = motorRates(1);
    w2 = motorRates(2);
    w3 = motorRates(3);
    w4 = motorRates(4);
    l = params.el;
    gamma = params.gamma;

    
    %ZYX
    R = Rz(t(1))*Ry(t(2))*Rx(t(3));
    N = [Rx(t(3))'*Ry(t(2))'*[0;0;1] Rx(t(3))'*[0;1;0] [1;0;0]]^(-1);

    odot = v;
    tdot = N*[p;q;r];
    
    vdot = 1/params.m*([0;0;params.m*params.g] + R*[0;0;-params.kF*(w1^2+w2^2+w3^2+w4^2)]);
   
    pdot = -1/JBx*(q*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) - l*(kF*w1^2 - kF*w2^2) - q*r*(JBy + 4*JPy));
    qdot = -1/JBy*(-p*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) - l*(kF*w3^2 - kF*w4^2) + p*r*(JBx + 4*JPx));
    rdot = -1/JBz*(- kF*kT*w1^2 - kF*kT*w2^2 + kF*kT*w3^2 + kF*kT*w4^2 + gamma*r - p*q*(JBx + 4*JPx) + p*q*(JBy + 4*JPy));
    wdot = [pdot;qdot;rdot];
    
    %motors don't instantaneously spin as fast as we want them to 
    motorRatesdot = 1/params.sigma_mot.*([u(1);u(2);u(3);0]-motorRates);
    
    xdot = [odot;tdot;vdot;wdot;motorRatesdot];

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
