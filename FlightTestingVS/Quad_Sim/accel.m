function out=accel(g,m,k_F,x)

% Measurement function for accelerometer simulation

F = sum(k_F.*x(13:16));
trueaccel = [0;0;g]+angle2dcm(x(9),x(8),x(7)).'*[0;0;-F]./m;
bias = 0;
sigma = 0.0;
out = trueaccel + randn(length(trueaccel),1).*sigma+bias;
