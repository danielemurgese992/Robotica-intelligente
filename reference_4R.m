function [qd, dqd, ddqd, Xd] = reference_4R(t, ref, p)
% REFERENCE_4R  Riferimento nello spazio dei giunti all'istante t.
%
%   [qd, dqd, ddqd, Xd] = reference_4R(t, ref, p)
%
%   qd   : da cinematica inversa analitica (ik_4R), riportata sul ramo
%          continuo pre-calcolato in ref (multipli di 2*pi)
%   dqd  : Ja^-1 * dX                         (Jacobiano aumentato 4x4)
%   ddqd : Ja^-1 * (ddX - dJa * dqd)
%   dove Ja = [J(1:2,:); 1 1 1 1; 1 1 1 0] lega dq a [dx dy dphi dpsi].
L = p.robot.L;
t = min(max(t, 0), ref.tEnd);
[Xd, dX, ddX] = task_reference_4R(t, p);
qd = ik_4R(Xd(1), Xd(2), Xd(3), L, p.ik.elbow, Xd(4));
qGrid = interp1(ref.t, ref.q.', t).';
qd = qd + 2*pi*round((qGrid - qd)/(2*pi));   % stesso ramo continuo
[J, ~] = jacobian_4R(qd, L);
Ja  = [J(1:2,:); 1 1 1 1; 1 1 1 0];
dqd = Ja \ dX;
[~, Jdot] = jacobian_4R(qd, L, dqd);
dJa = [Jdot(1:2,:); zeros(2,4)];
ddqd = Ja \ (ddX - dJa*dqd);
end
