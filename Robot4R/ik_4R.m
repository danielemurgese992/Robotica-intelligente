function [theta, info] = ik_4R(x, y, phi, L, elbow, psi)
% IK_4R  Cinematica inversa analitica del manipolatore planare 4R.
%
%   theta        = ik_4R(x, y, phi, L)
%   theta        = ik_4R(x, y, phi, L, elbow)
%   theta        = ik_4R(x, y, phi, L, elbow, psi)
%   [theta,info] = ik_4R(...)
%
%   Ingressi:
%     x, y   : posizione desiderata dell'end-effector              [m]
%     phi    : orientamento desiderato phi = th1+th2+th3+th4       [rad]
%     L      : lunghezze dei link [L1 L2 L3 L4]                    [m]
%     elbow  : 'up' (gomito alto, default) | 'down' (gomito basso)
%              (accetta anche 'alto' / 'basso')
%     psi    : parametro di ridondanza psi = th1+th2+th3, cioe' l'orientamento
%              assoluto del link 3 [rad]. Se omesso o vuoto si usa psi = phi
%              (theta4 = 0); se tale scelta non e' raggiungibile si cerca il
%              valore di psi raggiungibile piu' vicino a phi.
%   Uscite:
%     theta  : vettore 4x1 [th1; th2; th3; th4] (NaN se il punto non e'
%              raggiungibile)
%     info   : struttura con
%              .reachable  true/false
%              .psi        valore di psi effettivamente usato
%              .wrist      [xw yw]  punto del "polso" (a monte del link 4)
%              .p2         [x2 y2]  punto a valle del link 2
%              .solutions  4x2, colonne = soluzione gomito alto / gomito basso
%
%   Procedura:
%     1) polso:  xw = x - L4 cos(phi),  yw = y - L4 sin(phi)
%     2) gruppo 3R (th1,th2,th3) con orientamento psi del link 3:
%            x2 = xw - L3 cos(psi),   y2 = yw - L3 sin(psi)
%        problema 2R:  c2 = (x2^2+y2^2-L1^2-L2^2)/(2 L1 L2)
%                      s2 = -/+ sqrt(1-c2^2)      (gomito alto / basso)
%                      th2 = atan2(s2, c2)
%                      th1 = atan2(y2,x2) - atan2(L2 s2, L1 + L2 c2)
%                      th3 = psi - th1 - th2
%     3) quarto giunto:  th4 = phi - (th1 + th2 + th3) = phi - psi
if nargin < 5 || isempty(elbow), elbow = 'up'; end
L = L(:);
switch lower(elbow)
    case {'up', 'alto'},    sel = 1;
    case {'down', 'basso'}, sel = 2;
    otherwise, error('ik_4R:elbow', 'elbow deve essere ''up'' o ''down''.');
end

% 1) punto del polso
xw = x - L(4)*cos(phi);
yw = y - L(4)*sin(phi);

% 2) scelta del parametro di ridondanza psi
if nargin < 6 || isempty(psi)
    psi = phi;
    if ~reach2R(xw - L(3)*cos(psi), yw - L(3)*sin(psi), L)
        cand = phi + linspace(-pi, pi, 721);
        ok = false(size(cand));
        for k = 1:numel(cand)
            ok(k) = reach2R(xw - L(3)*cos(cand(k)), yw - L(3)*sin(cand(k)), L);
        end
        if any(ok)
            cand = cand(ok);
            [~, idx] = min(abs(cand - phi));
            psi = cand(idx);
        end
    end
end

% 3) problema 2R per theta1, theta2
x2 = xw - L(3)*cos(psi);
y2 = yw - L(3)*sin(psi);
c2 = (x2^2 + y2^2 - L(1)^2 - L(2)^2) / (2*L(1)*L(2));

info.psi   = psi;
info.wrist = [xw yw];
info.p2    = [x2 y2];

if abs(c2) > 1 + 1e-12
    info.reachable = false;
    info.solutions = nan(4,2);
    theta = nan(4,1);
    return
end
c2 = max(min(c2, 1), -1);
s2v = [-sqrt(1 - c2^2), +sqrt(1 - c2^2)];   % [gomito alto, gomito basso]
sols = zeros(4,2);
for k = 1:2
    s2  = s2v(k);
    th2 = atan2(s2, c2);
    th1 = atan2(y2, x2) - atan2(L(2)*s2, L(1) + L(2)*c2);
    th3 = psi - th1 - th2;
    th4 = phi - psi;
    sols(:,k) = wrapPi([th1; th2; th3; th4]);
end
info.reachable = true;
info.solutions = sols;
theta = sols(:, sel);
end

function ok = reach2R(x2, y2, L)
r = hypot(x2, y2);
ok = (r <= L(1) + L(2)) && (r >= abs(L(1) - L(2)));
end

function a = wrapPi(a)
a = mod(a + pi, 2*pi) - pi;
end
