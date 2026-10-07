function [tau, qd, dqd, e, de] = controller_4R(t, q, dq, z, ref, p)
% CONTROLLER_4R  Legge di controllo nello spazio dei giunti.
%
%   [tau, qd, dqd, e, de] = controller_4R(t, q, dq, z, ref, p)
%
%   e = qd - q,  de = dqd - dq,  z = integrale di e (stato del controllore)
%
%   p.ctrl.type = 'computed_torque' (coppia calcolata / inverse dynamics):
%       tau = M^(q) (ddqd + Kd de + Kp e + Ki z) + C^(q,dq) dq + G^(q)
%     Con modello perfetto la dinamica dell'errore diventa lineare e
%     disaccoppiata:  dde + Kd de + Kp e + Ki z = 0  (esponenzialmente
%     stabile se Kp, Kd > 0 e, con azione integrale, Kd Kp > Ki).
%     Guadagni: p.ctrl.Kp, p.ctrl.Kd, p.ctrl.Ki.
%
%   p.ctrl.type = 'pd_gravity' (PD con compensazione di gravita'):
%       tau = Kp e + Kd de + Ki z + G^(q)
%     Stabile (Lyapunov) per la regolazione; nell'inseguimento lascia un
%     errore residuo che diminuisce all'aumentare dei guadagni.
%     Guadagni: p.ctrl.pd.Kp, p.ctrl.pd.Kd, p.ctrl.pd.Ki.
%
%   M^, C^, G^ sono calcolati con masse e inerzie moltiplicate per
%   p.ctrl.modelScale (1 = modello esatto) per testare la robustezza.
q = q(:); dq = dq(:); z = z(:);
[qd, dqd, ddqd] = reference_4R(t, ref, p);
e  = qd - q;
de = dqd - dq;

rHat   = p.robot;
rHat.m = rHat.m * p.ctrl.modelScale;
rHat.I = rHat.I * p.ctrl.modelScale;

switch lower(p.ctrl.type)
    case 'computed_torque'
        [Mh, Ch, Gh] = dynamics_4R(q, dq, rHat);
        tau = Mh*(ddqd + p.ctrl.Kd*de + p.ctrl.Kp*e + p.ctrl.Ki*z) + Ch*dq + Gh;
    case 'pd_gravity'
        [~, ~, Gh] = dynamics_4R(q, dq, rHat);
        tau = p.ctrl.pd.Kp*e + p.ctrl.pd.Kd*de + p.ctrl.pd.Ki*z + Gh;
    otherwise
        error('controller_4R:type', 'p.ctrl.type deve essere ''computed_torque'' o ''pd_gravity''.');
end
tauMax = p.ctrl.tauMax(:);
tau = max(min(tau, tauMax), -tauMax);
end
