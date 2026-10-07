function S = derive_dynamics_4R_sym(p, doGenerate)
% DERIVE_DYNAMICS_4R_SYM  Derivazione simbolica (Eulero-Lagrange) di M, C, G.
%
%   S = derive_dynamics_4R_sym(p)
%   S = derive_dynamics_4R_sym(p, doGenerate)
%
%   Richiede Symbolic Math Toolbox (MATLAB) oppure il pacchetto "symbolic"
%   (GNU Octave). Passi eseguiti:
%     1) posizioni dei baricentri dalle matrici DH simboliche
%     2) energia cinetica  T = sum_i 1/2 m_i |v_ci|^2 + 1/2 I_i w_i^2
%     3) energia potenziale V = sum_i m_i g y_ci
%     4) M(q) = d^2 T / d dq^2 ,  G(q) = dV/dq
%     5) C(q,dq) tramite simboli di Christoffel
%     6) verifica: le equazioni di Lagrange d/dt(dL/d dq) - dL/dq calcolate
%        direttamente coincidono con M ddq + C dq + G
%     7) confronto numerico con dynamics_4R (formulazione chiusa) usando i
%        valori in p (default: params_4R)
%     8) (solo MATLAB, se doGenerate = true) genera con matlabFunction i file
%        M_4R_sym.m, C_4R_sym.m, G_4R_sym.m nella cartella "generated"
%
%   S contiene le espressioni simboliche (S.M, S.C, S.G, S.T, S.V) e
%   l'errore massimo del confronto numerico (S.maxErr).
if nargin < 1 || isempty(p), p = params_4R(); end
if nargin < 2, doGenerate = false; end
isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
if isOctave, pkg load symbolic; end
% semplificazione delle espressioni: rapida in MATLAB, molto lenta in Octave
if isOctave, simp = @(e) e; else, simp = @(e) simplify(e); end

syms q1 q2 q3 q4 dq1 dq2 dq3 dq4 ddq1 ddq2 ddq3 ddq4 real
syms L1 L2 L3 L4 lc1 lc2 lc3 lc4 m1 m2 m3 m4 I1 I2 I3 I4 g real
q   = [q1; q2; q3; q4];   dq = [dq1; dq2; dq3; dq4];  ddq = [ddq1; ddq2; ddq3; ddq4];
L   = [L1; L2; L3; L4];   lc = [lc1; lc2; lc3; lc4];
m   = [m1; m2; m3; m4];   I  = [I1; I2; I3; I4];

% 1) terne DH e baricentri
T = sym(eye(4));
o = sym(zeros(3,5));           % origini delle terne 0..4
for i = 1:4
    T = T * dh_matrix(L(i), 0, 0, q(i));
    o(:, i+1) = T(1:3, 4);
end
Tkin = sym(0); V = sym(0);
for i = 1:4
    pc  = o(:,i) + lc(i)/L(i) * (o(:,i+1) - o(:,i));   % baricentro del link i
    Jv  = jacobian(pc, q);
    vc  = Jv * dq;
    w   = sum(dq(1:i));                                % velocita' angolare (asse z)
    Tkin = Tkin + m(i)*(vc.'*vc)/2 + I(i)*w^2/2;
    V    = V + m(i)*g*pc(2);                            % gravita' lungo -y
end

fprintf('  [1-3] energie T e V calcolate\n');
% 4) matrice di inerzia e vettore gravitazionale
M = sym(zeros(4));
for k = 1:4
    for j = 1:4
        M(k,j) = simp(diff(diff(Tkin, dq(k)), dq(j)));
    end
end
G = sym(zeros(4,1));
for k = 1:4, G(k) = simp(diff(V, q(k))); end

fprintf('  [4] M(q) e G(q) calcolati\n');
% 5) Coriolis / centrifughi con i simboli di Christoffel
%    C_kj = sum_i 1/2 ( dM_kj/dq_i + dM_ki/dq_j - dM_ij/dq_k ) dq_i
%    In forma matriciale, detta D_i = dM/dq_i e V = [D_1 dq, ..., D_4 dq]:
%    C = 1/2 ( dM/dt + V - V' ),   con dM/dt = sum_i D_i dq_i
%    (equivalente al triplo ciclo ma con molte meno operazioni simboliche)
dMdq = jacobian(M(:), q);                  % 16x4: colonna i = vec(dM/dq_i)
Mdot = sym(zeros(4));  Vm = sym(zeros(4));
for i = 1:4
    Di = reshape(dMdq(:, i), 4, 4);
    Mdot = Mdot + Di*dq(i);
    Vm(:, i) = Di*dq;
end
C = (Mdot + Vm - Vm.')/2;
fprintf('  [5] C(q,dq) calcolata (Christoffel)\n');
% 6) verifica delle equazioni di Lagrange calcolate direttamente
Lag = Tkin - V;
tauLag = sym(zeros(4,1));
for k = 1:4
    dLdq_dot = diff(Lag, dq(k));
    ddt = jacobian(dLdq_dot, q)*dq + jacobian(dLdq_dot, dq)*ddq;   % derivata totale
    tauLag(k) = ddt - diff(Lag, q(k));
end
resid = tauLag - (M*ddq + C*dq + G);

% 7) confronto numerico con dynamics_4R
r = p.robot;
symVars = [L1 L2 L3 L4 lc1 lc2 lc3 lc4 m1 m2 m3 m4 I1 I2 I3 I4 g];
numVals = [r.L(:).' r.lc(:).' r.m(:).' r.I(:).' r.g];
maxErr = 0; maxRes = 0;
for trial = 1:3
    qv = 2*pi*rand(4,1) - pi; dqv = 2*rand(4,1) - 1; ddqv = 2*rand(4,1) - 1;
    vars = [symVars, q.', dq.', ddq.'];
    vals = [numVals, qv.', dqv.', ddqv.'];
    Ms = double(subs(M, vars, vals));
    Cs = double(subs(C, vars, vals));
    Gs = double(subs(G, vars, vals));
    Rs = double(subs(resid, vars, vals));
    [Mn, Cn, Gn] = dynamics_4R(qv, dqv, r);
    maxErr = max([maxErr, max(abs(Ms(:)-Mn(:))), max(abs(Cs(:)-Cn(:))), max(abs(Gs-Gn))]);
    maxRes = max(maxRes, max(abs(Rs)));
end
fprintf('Derivazione simbolica completata.\n');
fprintf('  residuo Lagrange diretto vs M ddq + C dq + G : %.3e\n', maxRes);
fprintf('  max |simbolico - dynamics_4R|                : %.3e\n', maxErr);

% 8) generazione automatica di funzioni (solo MATLAB)
if doGenerate
    if isOctave
        warning('Generazione file con matlabFunction disponibile solo in MATLAB.');
    else
        outDir = fullfile(fileparts(mfilename('fullpath')), 'generated');
        if ~exist(outDir, 'dir'), mkdir(outDir); end
        matlabFunction(M, 'File', fullfile(outDir, 'M_4R_sym'), 'Vars', {q, L, lc, m, I});
        matlabFunction(C, 'File', fullfile(outDir, 'C_4R_sym'), 'Vars', {q, dq, L, lc, m, I});
        matlabFunction(G, 'File', fullfile(outDir, 'G_4R_sym'), 'Vars', {q, L, lc, m, g});
        fprintf('  file generati in %s\n', outDir);
    end
end
S = struct('M', M, 'C', C, 'G', G, 'T', Tkin, 'V', V, 'maxErr', maxErr, 'maxResLagrange', maxRes);
end
