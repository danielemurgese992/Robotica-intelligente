function [Ekin, Epot] = energy_4R(theta, dtheta, params)
% ENERGY_4R  Energia cinetica e potenziale del 4R (per verifiche numeriche).
%
%   Ekin = 1/2 dq' M(q) dq
%   Epot = sum_i m_i g y_ci(q)    (y_ci quota del baricentro del link i)
if isfield(params, 'robot'), params = params.robot; end
theta = theta(:); dtheta = dtheta(:);
M = dynamics_4R(theta, zeros(4,1), params);
Ekin = 0.5 * dtheta.' * M * dtheta;
ph = cumsum(theta);
L = params.L(:); lc = params.lc(:); m = params.m(:);
Epot = 0;
for i = 1:4
    yci  = sum(L(1:i-1) .* sin(ph(1:i-1))) + lc(i)*sin(ph(i));
    Epot = Epot + m(i) * params.g * yci;
end
end
