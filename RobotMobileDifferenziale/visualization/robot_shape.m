function S = robot_shape(q, robot)
%ROBOT_SHAPE  Coordinate del disegno del robot nella configurazione q.
%   S.body (cerchio), S.wheelR / S.wheelL (rettangoli), S.head (segmento di prua)
R = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3)));
th = linspace(0, 2*pi, 40)';
body = robot.radius * [cos(th) sin(th)];
wl = robot.r; ww = 0.4 * robot.r;
wheel = [-wl -ww; wl -ww; wl ww; -wl ww];
S.body   = (R * body')' + q(1:2);
S.wheelR = (R * (wheel + [0 -robot.d/2])')' + q(1:2);
S.wheelL = (R * (wheel + [0  robot.d/2])')' + q(1:2);
S.head   = (R * [0 0; robot.radius 0]')' + q(1:2);
end
