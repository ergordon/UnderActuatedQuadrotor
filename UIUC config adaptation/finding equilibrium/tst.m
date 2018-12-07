%%

clc; clear; close all;

% vars(1) = epsilon;
% nx = vars(2);
% ny = vars(3);
% nz   = vars(4);
% p = vars(5);
% q = vars(6);
% r = vars(7);
% pdot = vars(8);
% qdot = vars(9);
% rdot = vars(10);
% w1 = vars(11);
% w2 = vars(12);
% w3 = vars(13);
% w4= vars(14);
               %[epsilon nx nty nz p q r pdot qdot rdot w1 w2 w3 w4]
% var_guess = [.03 0 .289 .958 0 5.69 18.89 0 0 0 500 500 250 0]';
var_guess = [.03 0 .289 .76 0 5.69 18.89 0 0 0 700 700 500 0]';

equilibRoot(var_guess)'

%%
clc; clear; close all;

%[epsilon nx ny nz p q r pdot qdot rdot w1 w2 w3 w4]
kF = 7.46e-6;
epsilon_eq = 0.039188 ;
 nx_eq = -0.105822 ;
 ny_eq = 0.000000 ;
 nz_eq = 0.994385 ;
 p_eq = -2.700386 ;
 q_eq = 0.000000 ;
 r_eq = 25.374955 ;
 w1_eq = 614.993890 ;
 w2_eq = 614.993890 ;
 w3_eq = 434.866350 ;
 w4_eq = 0.000000 ;
 
 
 f1_eq = kF*w1_eq^2;
 f2_eq = kF*w2_eq^2;
 f3_eq = kF*w3_eq^2;
 f4_eq = kF*w4_eq^2;
 
 
save('equilib.mat')
 
 % eq = struct;
    
% save('equilib.')

% eq.f1 = f1_eq;
% eq.f2 = f2_eq;
% eq.f3 = f3_eq;
% eq.f3 = f4_eq;
% eq.n = [nx_eq;ny_eq;nz_eq];
% eq.wB = [p_eq;q_eq;r_eq];
% eq.w1 = w1_eq;
% eq.w2 = w2_eq;
% eq.w3 = w3_eq;
% eq.w4 = w4_eq;
% 
% %equilibrium reduced attitude and input
% eq.s = [p_eq;q_eq;nx_eq;ny_eq]; %[p;q;nx;ny]
% eq.u = [f1_eq-f3_eq;f2_eq]; %equilibrium (force) input for inner loop reduced state
% eq.a = [0;0;0]; %equilibrium input (acceleration) for outter loop

