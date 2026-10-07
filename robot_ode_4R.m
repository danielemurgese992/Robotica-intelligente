function dx = robot_ode_4R(t, x, ref, p)
% ROBOT_ODE_4R  Equazioni di stato del robot in anello chiuso (per ode45).
%
%   Stato x = [q; dq; z] (12x1), con z = integrale dell'errore e = qd - q
%   (stato dell'azione integrale del controllore).
%   ddq = M(q) \ ( tau - C(q,dq) dq - G(q) ),   dz = e
%   con tau fornita da controller_4R e M, C, G del robot "reale" (p.robot).
q  = x(1:4);
dq = x(5:8);
z  = x(9:12);
[tau, ~, ~, e] = controller_4R(t, q, dq, z, ref, p);
[M, C, G] = dynamics_4R(q, dq, p.robot);
ddq = M \ (tau - C*dq - G);
dx = [dq; ddq; e];
end
