function [J, Jdot] = jacobian_4R(theta, L, dtheta)
% JACOBIAN_4R  Jacobiano analitico (planare) del 4R e sua derivata temporale.
%
%   [J, Jdot] = jacobian_4R(theta, L, dtheta)
%
%   J (3x4) lega le velocita' di giunto alle velocita' del compito:
%       [dx; dy; dphi] = J(theta) * dtheta
%   con, detti phi_k = theta1+...+thetak gli angoli assoluti dei link,
%       J(1,j) = -sum_{k>=j} L_k sin(phi_k)
%       J(2,j) =  sum_{k>=j} L_k cos(phi_k)
%       J(3,j) =  1
%   Jdot (3x4) e' la derivata temporale di J (richiede dtheta).
theta = theta(:); L = L(:);
if nargin < 3, dtheta = zeros(4,1); end
dtheta = dtheta(:);
phiAbs  = cumsum(theta);        % angoli assoluti dei link
dphiAbs = cumsum(dtheta);
J = zeros(3,4); Jdot = zeros(3,4);
for j = 1:4
    k = j:4;
    J(1,j)    = -sum(L(k) .* sin(phiAbs(k)));
    J(2,j)    =  sum(L(k) .* cos(phiAbs(k)));
    J(3,j)    =  1;
    Jdot(1,j) = -sum(L(k) .* cos(phiAbs(k)) .* dphiAbs(k));
    Jdot(2,j) = -sum(L(k) .* sin(phiAbs(k)) .* dphiAbs(k));
end
end
