function qn = integrate_step(q, wR, wL, robot, dt, method)
%INTEGRATE_STEP  Avanza lo stato di un passo dt con ingressi ruota costanti.
%   method: 'euler' (Eulero esplicito), 'rk4' (Runge-Kutta 4) oppure
%           'exact' (integrazione in forma chiusa lungo un arco di circonferenza).
f = @(qq) diffdrive_dynamics(qq, wR, wL, robot);
switch lower(method)
    case 'euler'
        qn = q + dt * f(q);
    case 'rk4'
        k1 = f(q);
        k2 = f(q + dt/2 * k1);
        k3 = f(q + dt/2 * k2);
        k4 = f(q + dt * k3);
        qn = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    case 'exact'
        [v, w] = wheels_to_unicycle(wR, wL, robot);
        th = q(3);
        if abs(w) < 1e-9
            qn = [q(1) + v*dt*cos(th), q(2) + v*dt*sin(th), th];
        else
            qn = [q(1) + v/w * (sin(th + w*dt) - sin(th)), ...
                  q(2) - v/w * (cos(th + w*dt) - cos(th)), ...
                  th + w*dt];
        end
    otherwise
        error('Integratore "%s" non valido.', method);
end
qn(3) = wrap_to_pi(qn(3));
end
