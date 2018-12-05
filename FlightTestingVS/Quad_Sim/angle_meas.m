function out=angle_meas(x)

% Measurement function for Euler angles simuation

bias = 0;
sigma = 0;
out = x(7:9)+randn(3,1).*sigma+bias;
