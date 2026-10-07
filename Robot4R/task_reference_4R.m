function [X, dX, ddX] = task_reference_4R(t, p)
% TASK_REFERENCE_4R  Traiettoria desiderata nello spazio operativo (aumentato).
%
%   [X, dX, ddX] = task_reference_4R(t, p)
%
%   X = [x; y; phi; psi] con derivate prima e seconda (analitiche).
%   - percorso geometrico: cerchio o segmento (p.traj.type)
%   - legge oraria: polinomio quintico s(t) in [0,1] su [0, T], con
%     velocita' e accelerazione nulle agli estremi; dopo T il punto resta fermo
%   - orientamento: phi(s) = phiStart + (phiEnd - phiStart) s
%   - ridondanza:   psi    = phi + p.ik.psiOffset   (psi = th1+th2+th3)
[s, ds, dds] = quintic_s(t, p.traj.T);
switch lower(p.traj.type)
    case 'circle'
        c = p.traj.center(:); r = p.traj.radius;
        k = 2*pi*p.traj.nTurns;
        a   = p.traj.theta0 + k*s;  da = k*ds;  dda = k*dds;
        pos = c + r*[cos(a); sin(a)];
        vel = r*[-sin(a); cos(a)]*da;
        acc = r*([-cos(a); -sin(a)]*da^2 + [-sin(a); cos(a)]*dda);
    case 'line'
        p0 = p.traj.pStart(:); p1 = p.traj.pEnd(:);
        pos = p0 + (p1 - p0)*s;
        vel = (p1 - p0)*ds;
        acc = (p1 - p0)*dds;
    otherwise
        error('task_reference_4R:type', 'p.traj.type deve essere ''circle'' o ''line''.');
end
dphi = p.traj.phiEnd - p.traj.phiStart;
phi  = p.traj.phiStart + dphi*s;
X   = [pos; phi; phi + p.ik.psiOffset];
dX  = [vel; dphi*ds;  dphi*ds];
ddX = [acc; dphi*dds; dphi*dds];
end

function [s, ds, dds] = quintic_s(t, T)
% legge oraria quintica: s = 10 tau^3 - 15 tau^4 + 6 tau^5,  tau = t/T
if t <= 0
    s = 0; ds = 0; dds = 0; return
elseif t >= T
    s = 1; ds = 0; dds = 0; return
end
tau = t/T;
s   = 10*tau^3 - 15*tau^4 + 6*tau^5;
ds  = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
dds = (60*tau - 180*tau^2 + 120*tau^3) / T^2;
end
