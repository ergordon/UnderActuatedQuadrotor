function sdot = getReducedRates(n,wB,w1,w2,w3,w4,u1,u2,params)
    
    JBx = params.JB(1,1);
    JBy = params.JB(2,2);
    JBz = params.JB(3,3);
    JPx = params.JP(1,1);
    JPy = params.JP(2,2);
    JPz = params.JP(3,3);
    kF = params.kF;
    kT = params.kT;
    l = params.el;
    
    p = wB(1);
    q = wB(2);
    r = wB(3);
    
    
    ndot = -cross(wB,n);
    
%     pdot = -1/JBx*(q*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) - l*u1 - q*r*(JBy + 4*JPy));
    pdot = -1/JBx*(q*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) + l*u1 - q*r*(JBy + 4*JPy));
    qdot = -1/JBy*(- p*(r*(JBz + 4*JPz) + JPz*(w1 + w2 + w3 + w4)) - l*u2 + l*kF*w4^2 + p*r*(JBx + 4*JPx));
    
    sdot = [pdot;qdot;ndot(1);ndot(2)];

end

