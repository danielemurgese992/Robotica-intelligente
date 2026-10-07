function res = simulate_4R(p)
% SIMULATE_4R  Simula il robot 4R in anello chiuso con ode45.
%
%   res = simulate_4R(p)
%
%   Uscita res (campionata ogni p.sim.dtOut):
%     res.t                 tempi                                  (1xN)
%     res.q, res.dq         posizioni / velocita' di giunto         (4xN)
%     res.qd, res.dqd       riferimenti di giunto                   (4xN)
%     res.e                 errore di posizione qd - q              (4xN)
%     res.tau               coppie ai giunti                        (4xN)
%     res.Xd                riferimento cartesiano [x;y;phi;psi]    (4xN)
%     res.Xee               posa reale dell'end-effector [x;y;phi]  (3xN)
%     res.ref               struttura della traiettoria pre-calcolata
ref = build_reference_4R(p);
q0  = ref.q(:,1) + p.sim.q0Offset(:);
x0  = [q0; p.sim.dq0(:); zeros(4,1)];   % [q; dq; integrale errore]
tspan = 0:p.sim.dtOut:ref.tEnd;
opts = odeset('RelTol', p.sim.RelTol, 'AbsTol', p.sim.AbsTol);
fprintf('Simulazione ode45 su [0, %.2f] s ... ', ref.tEnd);
tic;
[t, X] = ode45(@(t,x) robot_ode_4R(t, x, ref, p), tspan, x0, opts);
fprintf('completata in %.1f s.\n', toc);

N = numel(t);
res.t = t(:).';  res.q = X(:,1:4).';  res.dq = X(:,5:8).';  res.z = X(:,9:12).';
res.qd = zeros(4,N); res.dqd = zeros(4,N); res.e = zeros(4,N);
res.tau = zeros(4,N); res.Xd = zeros(4,N); res.Xee = zeros(3,N);
for k = 1:N
    [tau, qd, dqd, e] = controller_4R(t(k), res.q(:,k), res.dq(:,k), res.z(:,k), ref, p);
    res.tau(:,k) = tau; res.qd(:,k) = qd; res.dqd(:,k) = dqd; res.e(:,k) = e;
    res.Xd(:,k) = task_reference_4R(t(k), p);
    [~, x, y] = fk_4R(res.q(:,k), p.robot.L);
    res.Xee(:,k) = [x; y; sum(res.q(:,k))];
end
res.ref = ref;
ex = res.Xd(1:2,:) - res.Xee(1:2,:);
res.cartErr = sqrt(sum(ex.^2, 1));
fprintf('  errore cartesiano finale: %.2e m | max dopo il transitorio (t > T/2): %.2e m\n', ...
    res.cartErr(end), max(res.cartErr(res.t > p.traj.T/2)));
fprintf('  coppie massime |tau_i| [N m]: %s\n', mat2str(max(abs(res.tau),[],2).', 4));
end
