function ok = test_4R(p, runSymbolic)
% TEST_4R  Batteria di test numerici del progetto 4R.
%
%   ok = test_4R()                 usa params_4R()
%   ok = test_4R(p)                usa i parametri forniti
%   ok = test_4R(p, true)          include il confronto con la derivazione
%                                  simbolica (richiede Symbolic Toolbox)
%
%   Ogni test stampa PASS/FAIL e l'errore misurato; ok = true se tutti
%   i test sono superati.
if nargin < 1 || isempty(p), p = params_4R(); end
if nargin < 2, runSymbolic = false; end
L = p.robot.L(:); r = p.robot;
rng(1);                                   % riproducibilita'
nRand = 200;
results = {};

% ---------- 1. Cinematica diretta: confronto con formule in forma chiusa
err = 0;
for k = 1:nRand
    q = 2*pi*rand(4,1) - pi;
    [T, x, y, phi] = fk_4R(q, L);
    ph = cumsum(q);
    err = max([err, abs(x - sum(L.*cos(ph))), abs(y - sum(L.*sin(ph))), ...
               abs(angdiff(phi, sum(q))), abs(det(T(1:3,1:3)) - 1)]);
end
results(end+1,:) = {'FK: x, y, phi = forma chiusa', err, 1e-12};

% ---------- 2. Cinematica inversa: FK(IK(p)) = p (entrambi i gomiti, psi noto)
err = 0; errQ = 0;
for k = 1:nRand
    q = 2*pi*rand(4,1) - pi;
    [~, x, y, phi] = fk_4R(q, L);
    psi = sum(q(1:3));
    for el = {'up', 'down'}
        [qi, info] = ik_4R(x, y, phi, L, el{1}, psi);
        [~, xi, yi, phii] = fk_4R(qi, L);
        err = max([err, abs(xi-x), abs(yi-y), abs(angdiff(phii, phi))]);
        if ~info.reachable, err = Inf; end
    end
    % una delle due soluzioni deve coincidere con q di partenza
    [~, info] = ik_4R(x, y, phi, L, 'up', psi);
    d = min(max(abs(angdiff(info.solutions(:,1), q))), max(abs(angdiff(info.solutions(:,2), q))));
    errQ = max(errQ, d);
end
results(end+1,:) = {'IK: FK(IK(p)) - p  (gomito alto/basso)', err, 1e-9};
results(end+1,:) = {'IK: q originale tra le 2 soluzioni', errQ, 1e-7};

% ---------- 3. IK con psi automatico (default)
err = 0;
for k = 1:nRand
    q = 2*pi*rand(4,1) - pi;
    [~, x, y, phi] = fk_4R(q, L);
    qi = ik_4R(x, y, phi, L);
    [~, xi, yi, phii] = fk_4R(qi, L);
    err = max([err, abs(xi-x), abs(yi-y), abs(angdiff(phii, phi))]);
end
results(end+1,:) = {'IK: FK(IK(p)) - p  (psi automatico)', err, 1e-9};

% ---------- 4. Punto non raggiungibile
[qi, info] = ik_4R(2*sum(L), 0, 0, L);
results(end+1,:) = {'IK: punto fuori dallo spazio di lavoro', double(info.reachable || ~all(isnan(qi))), 0.5};

% ---------- 5. Jacobiano e derivata vs differenze finite
err = 0; h = 1e-6;
for k = 1:20
    q = 2*pi*rand(4,1) - pi; dq = 2*rand(4,1) - 1;
    [J, Jdot] = jacobian_4R(q, L, dq);
    Jn = zeros(3,4);
    for j = 1:4
        e = zeros(4,1); e(j) = h;
        [~, xp, yp, pp] = fk_4R(q+e, L); [~, xm, ym, pm] = fk_4R(q-e, L);
        Jn(:,j) = [xp-xm; yp-ym; angdiff(pp, pm)]/(2*h);
    end
    Jdn = (jacobian_4R(q + h*dq, L) - jacobian_4R(q - h*dq, L))/(2*h);
    err = max([err, max(abs(J(:)-Jn(:))), max(abs(Jdot(:)-Jdn(:)))]);
end
results(end+1,:) = {'Jacobiano e dJ/dt vs differenze finite', err, 1e-6};

% ---------- 6. M(q) simmetrica e definita positiva
err = 0; minEig = Inf;
for k = 1:nRand
    q = 2*pi*rand(4,1) - pi;
    M = dynamics_4R(q, zeros(4,1), r);
    err = max(err, max(max(abs(M - M.'))));
    minEig = min(minEig, min(eig(M)));
end
results(end+1,:) = {'M(q) simmetrica', err, 1e-12};
results(end+1,:) = {'M(q) definita positiva (-min autovalore)', -minEig, 0};

% ---------- 7. dM/dt - 2C antisimmetrica (proprieta' di passivita')
err = 0;
for k = 1:20
    q = 2*pi*rand(4,1) - pi; dq = 2*rand(4,1) - 1;
    [~, C] = dynamics_4R(q, dq, r);
    Mdot = (dynamics_4R(q + h*dq, dq, r) - dynamics_4R(q - h*dq, dq, r))/(2*h);
    N = Mdot - 2*C;
    err = max(err, max(max(abs(N + N.'))));
end
results(end+1,:) = {'dM/dt - 2C antisimmetrica', err, 1e-6};

% ---------- 8. G(q) = gradiente dell'energia potenziale
err = 0;
for k = 1:20
    q = 2*pi*rand(4,1) - pi;
    [~, ~, G] = dynamics_4R(q, zeros(4,1), r);
    Gn = zeros(4,1);
    for j = 1:4
        e = zeros(4,1); e(j) = h;
        [~, Vp] = energy_4R(q+e, zeros(4,1), r); [~, Vm] = energy_4R(q-e, zeros(4,1), r);
        Gn(j) = (Vp - Vm)/(2*h);
    end
    err = max(err, max(abs(G - Gn)));
end
results(end+1,:) = {'G(q) = dV/dq', err, 1e-6};

% ---------- 9. Conservazione dell'energia in evoluzione libera (tau = 0)
x0 = [deg2rad([30 -20 40 10]).'; zeros(4,1)];
f  = @(t,x) [x(5:8); freeAcc(x, r)];
[~, X] = ode45(f, [0 2], x0, odeset('RelTol', 1e-10, 'AbsTol', 1e-12));
E = zeros(size(X,1),1);
for k = 1:size(X,1)
    [Ek, Ep] = energy_4R(X(k,1:4), X(k,5:8), r); E(k) = Ek + Ep;
end
results(end+1,:) = {'Energia costante con tau = 0 (deriva)', max(abs(E - E(1))), 1e-5};

% ---------- 10. Inseguimento con coppia calcolata e modello esatto
pt = p;
pt.ctrl.type = 'computed_torque'; pt.ctrl.modelScale = 1; pt.ctrl.tauMax = Inf;
pt.traj.T = 3; pt.traj.tHold = 0.5; pt.sim.q0Offset = zeros(1,4); pt.sim.dq0 = zeros(1,4);
pt.sim.dtOut = 0.05;
res = simulate_4R(pt);
results(end+1,:) = {'Coppia calcolata: max errore cartesiano', max(res.cartErr), 1e-4};

% ---------- 11. (opzionale) confronto con derivazione simbolica
if runSymbolic
    S = derive_dynamics_4R_sym(p);
    results(end+1,:) = {'Simbolico vs dynamics_4R', S.maxErr, 1e-9};
    results(end+1,:) = {'Lagrange diretto vs M ddq + C dq + G', S.maxResLagrange, 1e-9};
end

% ---------- report
fprintf('\n%-46s %12s %10s  %s\n', 'TEST', 'ERRORE', 'SOGLIA', 'ESITO');
fprintf('%s\n', repmat('-', 1, 80));
ok = true;
for k = 1:size(results,1)
    pass = results{k,2} <= results{k,3};
    ok = ok && pass;
    if pass, s = 'PASS'; else, s = 'FAIL'; end
    fprintf('%-46s %12.3e %10.1e  %s\n', results{k,1}, results{k,2}, results{k,3}, s);
end
fprintf('%s\n', repmat('-', 1, 80));
if ok, fprintf('TUTTI I TEST SUPERATI\n\n'); else, fprintf('ALCUNI TEST NON SUPERATI\n\n'); end
end

function ddq = freeAcc(x, r)
[M, C, G] = dynamics_4R(x(1:4), x(5:8), r);
ddq = M \ (-C*x(5:8) - G);
end

function d = angdiff(a, b)
d = mod(a - b + pi, 2*pi) - pi;
end
