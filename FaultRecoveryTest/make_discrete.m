function [Ad, Bd] = make_discrete(Ac, Bc, dt)

    Ad = expm(Ac*dt);
    fun = @(t) expm(Ac*t)*Bc;
    Bd = integral(fun,0,dt,'ArrayValue',true);

end