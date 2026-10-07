function [U, gx, gy] = apf_potential(P, goal, env, prm, virt)
%APF_POTENTIAL  Potenziale artificiale totale e suo gradiente.
%
%   Attrattivo (quadratico vicino al goal, conico lontano):
%     d <= d*: Ua = 1/2 ka d^2                 grad = ka (p - pg)
%     d >  d*: Ua = d* ka d - 1/2 ka d*^2       grad = d* ka (p - pg)/d
%   Repulsivo (Khatib) per ogni ostacolo e bordo, con rho = clearance - inflation:
%     rho <= rho0: Ur = 1/2 kr (1/rho - 1/rho0)^2
%     grad Ur = -kr (1/rho - 1/rho0) / rho^2 * grad(rho)
%   Correzione GNRON (Ge & Cui): se d < R = goal_rep_radius il potenziale
%   repulsivo e' moltiplicato per f = (d/R)^n, cosi' il goal resta il minimo
%   globale anche quando e' vicino a ostacoli o bordi:
%     grad(f Ur) = f grad(Ur) + Ur grad(f),  grad(f) = n d^(n-2) (p - pg) / R^n
%   Ostacoli virtuali (opzionali, virt = K x 2 punti dei minimi locali gia'
%   visitati): stesso potenziale di Khatib con guadagno k_virtual e raggio
%   rho_virtual, usato per "riempire" i pozzi dei minimi locali.
%   P: N x 2 punti; U: N x 1; gx, gy: componenti del gradiente.
if nargin < 5, virt = zeros(0, 2); end
dx = P(:, 1) - goal(1); dy = P(:, 2) - goal(2);
d = sqrt(dx.^2 + dy.^2);
ka = prm.k_att; ds = prm.d_star;
near = d <= ds;
U = zeros(size(d)); gx = U; gy = U;
U(near)  = 0.5 * ka * d(near).^2;
gx(near) = ka * dx(near); gy(near) = ka * dy(near);
far = ~near;
U(far)  = ds * ka * d(far) - 0.5 * ka * ds^2;
gx(far) = ds * ka * dx(far) ./ d(far);
gy(far) = ds * ka * dy(far) ./ d(far);

[D, Gx, Gy] = obstacle_distances(P, env);
rho = max(D - env.inflation, 1e-3);
act = rho <= prm.rho0;
inv = (1 ./ rho - 1 / prm.rho0) .* act;
Ur = sum(0.5 * prm.k_rep * inv.^2, 2);
coef = -prm.k_rep * inv ./ rho.^2;
grx = sum(coef .* Gx, 2);
gry = sum(coef .* Gy, 2);
f = ones(size(d)); dfx = zeros(size(d)); dfy = dfx;
if prm.goal_rep_n > 0
    R = prm.goal_rep_radius; n = prm.goal_rep_n;
    in = d < R;
    f(in) = (d(in) / R).^n;
    dd = max(d(in), 1e-9);
    dfx(in) = n * dd.^(n - 2) .* dx(in) / R^n;
    dfy(in) = n * dd.^(n - 2) .* dy(in) / R^n;
end
U  = U + f .* Ur;
gx = gx + f .* grx + Ur .* dfx;
gy = gy + f .* gry + Ur .* dfy;

if ~isempty(virt) && prm.k_virtual > 0
    for k = 1:size(virt, 1)
        vx = P(:, 1) - virt(k, 1); vy = P(:, 2) - virt(k, 2);
        r = max(sqrt(vx.^2 + vy.^2), 1e-3);
        a = r <= prm.rho_virtual;
        iv = (1 ./ r - 1 / prm.rho_virtual) .* a;
        U = U + 0.5 * prm.k_virtual * iv.^2;
        cf = -prm.k_virtual * iv ./ r.^2;
        gx = gx + cf .* vx ./ r;
        gy = gy + cf .* vy ./ r;
    end
end
end
