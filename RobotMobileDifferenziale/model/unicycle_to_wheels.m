function [wR, wL] = unicycle_to_wheels(v, w, robot)
%UNICYCLE_TO_WHEELS  Cinematica inversa: (v, w) -> velocita' angolari delle ruote.
%   wR = (2v + w d) / (2r),   wL = (2v - w d) / (2r)
wR = (2 .* v + w .* robot.d) ./ (2 * robot.r);
wL = (2 .* v - w .* robot.d) ./ (2 * robot.r);
end
