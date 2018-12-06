%%

%clc; clear; close all;

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


%  equilibRoot(sol);
 fun = @equilibRoot;
 sol = fsolve(fun,sol);


sprintf('epsilon_eq=%f \n nx_eq=%f \n ny_eq=%f \n nz_eq=%f \n p_eq=%f \n q_eq=%f \n r_eq=%f \n pd_eq=%f \n qd_eq=%f \n rd_eq=%f \n w1_eq=%f \n w2_eq=%f \n w3_eq=%f \n w4_eq=%f \n',sol)