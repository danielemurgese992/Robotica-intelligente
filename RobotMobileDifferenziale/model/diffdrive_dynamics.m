function qdot = diffdrive_dynamics(q, wR, wL, robot)
%DIFFDRIVE_DYNAMICS  Modello cinematico differenziale  qdot = f(q, wR, wL).
%   q = [x y theta];  xdot = v cos(theta), ydot = v sin(theta), thetadot = w
[v, w] = wheels_to_unicycle(wR, wL, robot);
qdot = [v * cos(q(3)), v * sin(q(3)), w];
end
