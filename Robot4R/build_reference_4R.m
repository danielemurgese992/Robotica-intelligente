function ref = build_reference_4R(p)
% BUILD_REFERENCE_4R  Pre-calcola e valida la traiettoria di riferimento.
%
%   ref = build_reference_4R(p)
%
%   Campiona la traiettoria cartesiana su una griglia fine, risolve la
%   cinematica inversa in ogni punto (gomito p.ik.elbow, psi da p.traj),
%   verifica che tutti i punti siano raggiungibili e lontani da singolarita'
%   e "srotola" (unwrap) gli angoli per avere q_d(t) continua.
%   La struttura ref e' poi usata da reference_4R durante la simulazione.
L = p.robot.L;
ref.tEnd = p.traj.T + p.traj.tHold;
ref.t = linspace(0, ref.tEnd, max(2001, ceil(ref.tEnd/1e-3)+1));
N = numel(ref.t);
ref.q = zeros(4,N); ref.X = zeros(4,N); ref.sin2 = zeros(1,N);
for k = 1:N
    X = task_reference_4R(ref.t(k), p);
    [q, info] = ik_4R(X(1), X(2), X(3), L, p.ik.elbow, X(4));
    if ~info.reachable
        error('build_reference_4R:unreachable', ...
            ['Punto non raggiungibile a t = %.3f s: (x,y,phi) = (%.3f, %.3f, %.1f deg).\n' ...
             'Modificare la traiettoria (p.traj) o psiOffset (p.ik.psiOffset).'], ...
             ref.t(k), X(1), X(2), rad2deg(X(3)));
    end
    ref.q(:,k) = q;  ref.X(:,k) = X;  ref.sin2(k) = abs(sin(q(2)));
end
ref.q = unwrap(ref.q, [], 2);
if min(ref.sin2) < 0.05
    warning('build_reference_4R:singular', ...
        'La traiettoria passa vicino a una singolarita'' (|sin th2| min = %.3f).', min(ref.sin2));
end
end
