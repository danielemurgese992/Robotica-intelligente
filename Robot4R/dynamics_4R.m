function [M, C, G] = dynamics_4R(theta, dtheta, params)
% DYNAMICS_4R  Modello dinamico (Eulero-Lagrange) del manipolatore planare 4R.
%
%   [M, C, G] = dynamics_4R(theta, dtheta, params)
%
%   M(q) ddq + C(q,dq) dq + G(q) = tau
%
%   Ingressi:
%     theta, dtheta : posizioni [rad] e velocita' [rad/s] di giunto (4x1)
%     params        : struttura con i campi L, lc, m, I, g (vedi params_4R:
%                     si puo' passare p.robot). Se params e' la struttura
%                     completa p, viene usato automaticamente p.robot.
%   Uscite:
%     M : matrice di inerzia 4x4 (simmetrica, definita positiva)
%     C : matrice di Coriolis/centrifughi 4x4 (simboli di Christoffel,
%         quindi dM/dt - 2C e' antisimmetrica)
%     G : vettore gravitazionale 4x1
%
%   Formulazione chiusa (verificata contro la derivazione simbolica in
%   derive_dynamics_4R_sym.m). Con phi_k = th1+...+thk (angoli assoluti),
%   a_ik = L_k se k < i, a_ik = lc_i se k = i, e S matrice triangolare
%   inferiore di 1 (dphi = S*dq):
%     B_kl  = sum_{i>=max(k,l)} m_i a_ik a_il
%     Mb_kl = B_kl cos(phi_k - phi_l) + delta_kl I_k       -> M = S' Mb S
%     Cb_kl = B_kl sin(phi_k - phi_l) dphi_l               -> C = S' Cb S
%     Gb_k  = g cos(phi_k) sum_{i>=k} m_i a_ik             -> G = S' Gb
if isfield(params, 'robot'), params = params.robot; end
L = params.L(:); lc = params.lc(:); m = params.m(:); I = params.I(:); g = params.g;
theta = theta(:); dtheta = dtheta(:);
n = 4;
S = tril(ones(n));
ph  = S*theta;        % angoli assoluti
dph = S*dtheta;

% matrice dei "bracci" a(i,k)
a = zeros(n);
for i = 1:n
    a(i,1:i-1) = L(1:i-1).';
    a(i,i)     = lc(i);
end
% B_kl = sum_i m_i a_ik a_il   (a_ik = 0 per k > i)
B = a.' * diag(m) * a;

D  = ph - ph.';                 % D(k,l) = phi_k - phi_l
Mb = B .* cos(D) + diag(I);
Cb = (B .* sin(D)) .* (ones(n,1) * dph.');   % Cb(k,l) = B_kl sin(.) dphi_l
Gb = g * cos(ph) .* (a.' * m);

M = S.' * Mb * S;
C = S.' * Cb * S;
G = S.' * Gb;
M = (M + M.')/2;    % simmetria numerica
end
