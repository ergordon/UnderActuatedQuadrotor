function anglerates=gyro(x)

% Measurement function for gyro simulation

truerates = x(10:12);
bias = 0;
sigma = 0.0;
anglerates = truerates + randn(length(truerates),1).*sigma+bias;
