function states=MoCap(x)

% Measurement function for MoCap simulation. Outputs [x;y;z] and
% [phi;theta;psi] in the MoCap world frame.

% Correction... the python code attached the groundstation is going to give
% us derivatives. We'll assume they look really good...?

truestates=x;
bias = 0;
sigma = 0.0;
states = truestates + randn(length(truestates),1).*sigma+bias;
