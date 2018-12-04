function a = angle_diff(a1,a2)
a = mod((a1-a2) + 3*pi, 2*pi) - pi;
end