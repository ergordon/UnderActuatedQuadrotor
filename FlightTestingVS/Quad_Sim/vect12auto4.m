function out=vect12auto4(f,u,dist,t,x)
% This function evaluates an autonomous system state vector length 12 and
% control vector length 4. This file is a bit deprecated, but it allows us
% to put limits on control inputs... I think.

you=u(t,x);

kF=7.1e-6;
kM=1.3e-7;
l=0.17;
twolkF=1/(2*l*kF);
fourkF=1/(4*kF);
fourkM=1/(4*kM);
W_inv=[0 twolkF -fourkM fourkF;
    -twolkF 0 fourkM fourkF;
    0 -twolkF -fourkM fourkF;
    twolkF 0 fourkM fourkF];
%?W_inv=W.';
omega_cmd=W_inv*you;
omega=sign(omega_cmd).*sqrt(abs(omega_cmd));
cmd=0.2352*omega-24.6266;
cmd=min(max(cmd,1),200);
omega=(cmd+24.6266)/0.2352;
omega_cmd=sign(omega).*omega.^2;
you=W_inv\omega_cmd;

out=f(t,x,you);

% Force Disturbance
out=dist(t,out);

% % Upper/Lower limits on... we actually have no idea anymore. I might be
% % able to do limits on control... maybe.
% lb=125.7;
% ub=816.8;
% for i=13:16
%     if x(i)>=ub && out(i)>0
%         out(i)=0;
%     elseif x(i)<=lb && out(i)<0
%         out(i)=0;
%     end
% end

end
