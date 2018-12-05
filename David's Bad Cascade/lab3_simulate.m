function [t, state] = lab3_simulate

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Change parameter values and sample time to match your quadrotor (see
    %     your results from Labs #1 and #2)
    %   - Change initial time, final time, and initial conditions as you like
    %

    % Parameters
    g = 9.81;                 % acceleration of gravity
    m = 0.715;                  % mass
    J = diag([4093e-6, 3944e-6, 7593e-6]);    % moment of inertia matrix in body frame
    l = 0.17;                  % spar length
    kF = 7.46e-6;              % aerodynamic force coefficient
    kM = 1.23e-7;              % aerodynamic torque coefficient
    sigmamax = 1e3;         % maximum spin rate

    % Initial time
    t0 = 0;
    
   

    %initial conditions
%     o0 = [-.0313; 0.0074; -.0853];
    o0 = [2; 2; -3];
    theta0 = [1; 1; 1]*0;
    v0 = [0; 0; 0];
    w0 = [0; 0; 0];
    
    % Sample time
    dt_o = 1/50;
    dt_i = 1/500;
    % Final time
    t1 = 5;

    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Design a control policy (see HW2.5.2):
    %
    %       (xe, ue)    equilibrium point
    %       K           gain matrix
    %
      
    %all share the same A matrix
    A = [0 1;
         0 0];
    
    %use same cost for everything for now
%     Q = diag([1 1]);
%     R = diag([1]);
    
    %try to make a low penalty for being off position but a high penalty
    %for being off angle
    Q_i = diag([2 .1]);
    R_i = diag([5]);
    Q_o = diag([2 .1]);
    R_o = diag([5]);
    
    B = [0;-g];
    [Ax,Bx] = make_discrete(A,B,dt_o);
    Kx = dlqr(Ax, Bx, Q_o, R_o);
   
    B = [0;g];
   [Ay,By] = make_discrete(A,B,dt_o);
    Ky = dlqr(Ay, By, Q_o, R_o);
   
   B = [0;-1/m];
   [Az,Bz] = make_discrete(A,B,dt_o);
    Kz = dlqr(Az, Bz, Q_o, R_o);
    
    
    B = [0;1/J(3,3)];
    [At1,Bt1] = make_discrete(A,B,dt_i);
    Kth1 = dlqr(At1, Bt1, Q_i, R_i);
   
    B = [0;1/J(2,2)];
   [At2,Bt2] = make_discrete(A,B,dt_i);
    Kth2 = dlqr(At2, Bt2, Q_i, R_i);
   
   B = [0;1/J(1,1)];
   [At3,Bt3] = make_discrete(A,B,dt_i);
    Kth3 = dlqr(At3, Bt3, Q_i, R_i);
    
    
    
    %equilibriums for hover
    x_xdot_eq = [0;0];
    y_ydot_eq = [0;0];
    z_zdot_eq = [-1;0];
    th1_th1dot_eq = [0;0];
    th2_th2dot_eq = [0;0];
    th3_th3dot_eq = [0;0];
   
    ux_eq = 0;
    uy_eq = 0;
    uz_eq = m*g;
    
    
    uth1_eq = 0;
    uth2_eq = 0;
    uth3_eq = 0;
    

    % Create variables to keep track of time, state, input, and desired position
    t = [t0];
    state = [o0; theta0; v0; w0];
    u = [];
    
    
    %store the time and state at the end of very ode call
    %start with initial conditions
    last_t = t0;
    last_x = [o0; theta0; v0; w0];
    

    %outter loop
    tic
    for i = 1:((t1-t0)/dt_o)
        
        ti = last_t;
        xi = last_x;
        

        
        x_xdot = [xi(1);xi(7)];
        y_ydot = [xi(2);xi(8)];
        z_zdot = [xi(3);xi(9)];
       
        %get setpoint for inner loop
        %u = ...
        
        
        %get x from end of inner loop
        ux = -Kx*(x_xdot-x_xdot_eq);% + ux_eq; %ux is theta2 set point
        uy = -Ky*(y_ydot-y_ydot_eq);% + uy_eq; %uy is theta3 set point
        uz = -Kz*(z_zdot-z_zdot_eq) + uz_eq;  %uz is u4 ??set point??   (total force in z direction)
        
        
        %start and stop times for the inner loop
        t0_outter = t0 + (i-1)*dt_o;
        t1_outter = t0_outter + dt_o;
        
        
        %inner loop
        for j = 1:((t1_outter-t0_outter)/dt_i)
            
            tj = last_t;
            xj = last_x;
            
            
            th1_th1dot = [xj(4);xj(10)]; 
            th2_th2dot = [xj(5);xj(11)]; 
            th3_th3dot = [xj(6);xj(12)]; 
            

            uth1 = -Kth1*(th1_th1dot-th1_th1dot_eq) + uth1_eq; %uth1 is u3
            uth2 = -Kth2*(th2_th2dot-[ux;0]) + uth2_eq; %uth2 is u2
            uth3 = -Kth3*(th3_th3dot-[uy;0]) + uth3_eq; %uth3 is u1
            
             %try this in the inner loop?
%             z_zdot = [xj(3);xj(9)];
%             uz = -Kz*(z_zdot-z_zdot_eq) + uz_eq;  %uz is u4 ??set point??   (total force in z direction)
            
%             %try this in the inner loop?
%             z_zdot = [xj(3);xj(9)];
%             uz = -Kz*(z_zdot-z_zdot_eq) + uz_eq;  %uz is u4 ??set point??   (total force in z direction)
            
            u_desired = [uth1; uth2; uth3; uz];
%             u_desired = [uth1; uth2; uth3; uz]; % try switching th1 and th3?
            uj = GetBoundedInputs( u_desired , kF, kM, l, sigmamax);
            
            %have to keep track of the current X to send back to the inner and outter looops    
            
            
            % Get time and state at start of (i+1)'th sample interval
            [tsol, xsol] = ode45(@(t, x) h(t, x, uj, g, m, J), [tj tj+dt_i], xj);
            t(:, end+1) = tsol(end, :)';
            state(:, end+1) = xsol(end, :)';
            
            last_t = tsol(end, :)';
            last_x = xsol(end, :)';
        end
      end    
    toc
end

function xdot = h(t, x, u, g, m, J)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %   MODIFY
    %
    %   - Compute xdot given x, u, and other parameters (see HW2.2)

    
    o = x(1:3);
    t = x(4:6);
    
    theta_3=x(6);
    theta_2=x(5);
    theta_1=x(4);
    p=x(10);
    q=x(11);
    r=x(12);
    
    v = x(7:9);
    w = x(10:12);
    
    what = [0 -w(3) w(2);
            w(3) 0 -w(1);
            -w(2) w(1) 0];
    %ZYX
    R = Rz(t(1))*Ry(t(2))*Rx(t(3));
    
    N = [Rx(t(3))'*Ry(t(2))'*[0;0;1] Rx(t(3))'*[0;1;0] [1;0;0]]^(-1);
    
    odot = v;
    
    vdot = 1/m*([0;0;m*g] + R*[0;0;-u(4)]);
    wdot = J^(-1)*(u(1:3) - what*J*w);
    
    m=[0, sin(theta_3)/cos(theta_2), cos(theta_3)/cos(theta_2); 0, cos(theta_3), -sin(theta_3);...
           1, tan(theta_2)*sin(theta_3), tan(theta_2)*cos(theta_3)]*[p;q;r];
       
    tdot = [m(3);m(2);m(1)];%N*w;
    
    
    
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