function varargout=discretize(inner,outer,tspan,dt_i,dt_o,t,x)

persistent u_i u_o T_i T_o step_i step_o

% Hold onto control history. Only update every dt. If two outputs are
% asked for, spit back the control history and the related time vector. If
% only one output, give current control.

% Update: Need to keep both inner and outer loop control... and times. Now,
% it's either one output or four outputs. Come back sometime and generalize
% this for any level heirarchical controller.

if strcmp(class(inner),'function_handle')
    if isempty(u_i)
        u_int=outer(t,x);
        u_out=inner(t,x,u_int);
        T_i=tspan(1):dt_i:(tspan(2)+dt_i);
        u_i=zeros(length(u_out),length(T_i));
        u_i(:,1)=u_out;
        T_o=tspan(1):dt_o:(tspan(2)+dt_o);
        u_o=zeros(length(u_int),length(T_o));
        u_o(:,1)=u_int;
        step_i=2;
        step_o=2;
    else
        if t>=T_o(step_o)
            u_o(:,step_o)=outer(t,x);
            step_o=step_o+1;
        end
        if t>=T_i(step_i)
            u_out=inner(t,x,u_o(:,step_o-1));
            u_i(:,step_i)=u_out;
            step_i=step_i+1;
        else
            u_out=u_i(:,step_i-1);
        end
    end
end

if nargout==1
    varargout={u_out};
else
    varargout{1}=T_i(:,1:end-1);
    varargout{2}=u_i(:,1:end-1);
    varargout{3}=T_o(:,1:end-1);
    varargout{4}=u_o(:,1:end-1);
end

