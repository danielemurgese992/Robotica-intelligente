function [v, w, rho, alpha] = controller_rho_alpha(q, target, ctrl)
%CONTROLLER_RHO_ALPHA  Controllore cinematico polare verso un punto obiettivo.
%   rho   = distanza tra robot e target
%   alpha = angolo tra l'asse del robot e la direzione del target, in (-pi, pi]
%   v = k_rho * rho,   w = k_alpha * alpha
%   Con ctrl.cos_alpha_scaling = true:  v = k_rho * rho * max(cos(alpha), 0)
%   (il robot rallenta mentre si orienta e non avanza se il target e' dietro).
dx = target(1) - q(1); dy = target(2) - q(2);
rho = sqrt(dx^2 + dy^2);
alpha = wrap_to_pi(atan2(dy, dx) - q(3));
v = ctrl.k_rho * rho;
if ctrl.cos_alpha_scaling, v = v * max(cos(alpha), 0); end
w = ctrl.k_alpha * alpha;
end
