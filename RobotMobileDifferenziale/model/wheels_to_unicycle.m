function [v, w] = wheels_to_unicycle(wR, wL, robot)
%WHEELS_TO_UNICYCLE  Cinematica diretta del robot differenziale.
%   v = r/2 (wR + wL),   w = r/d (wR - wL)
v = robot.r / 2 .* (wR + wL);
w = robot.r / robot.d .* (wR - wL);
end
