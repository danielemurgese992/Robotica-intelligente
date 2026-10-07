function [path, info] = plan_apf(env, q0, qf, prm, coll_step)
%PLAN_APF  Pianificazione con campi di potenziale artificiali.
%   Discesa del gradiente a passo normalizzato:  p_{k+1} = p_k - step * gradU/|gradU|
%   Minimo locale: rilevato quando lo spostamento negli ultimi stall_window
%   passi e' minore di stall_dist (o il gradiente e' quasi nullo). Se
%   prm.escape = true:
%     - il minimo viene marcato con un ostacolo virtuale repulsivo (riempie il
%       pozzo, cosi' la discesa successiva non vi ricade);
%     - si esegue un random walk collision-free la cui lunghezza raddoppia a
%       ogni fuga (fino a escape_growth_max volte), poi si riprende la discesa.
%   Con prm.escape = false la pianificazione fallisce e il minimo e' segnalato.
goal = qf(1:2); p = q0(1:2);
cap = prm.max_iter + prm.max_escapes * prm.escape_steps * prm.escape_growth_max + 2;
path = zeros(cap, 2); n = 1; path(1, :) = p;
minima = zeros(0, 2); n_esc = 0; success = false; last_stall = 0;
for it = 1:prm.max_iter
    if norm(p - goal) < prm.goal_tol
        n = n + 1; path(n, :) = goal; success = true; break;
    end
    [~, gx, gy] = apf_potential(p, goal, env, prm, minima);
    g = [gx gy]; ng = norm(g);
    stalled = ng < 1e-6;
    if ~stalled && n > prm.stall_window && n - last_stall > prm.stall_window
        stalled = norm(path(n, :) - path(n - prm.stall_window, :)) < prm.stall_dist;
    end
    if stalled
        minima(end+1, :) = p; %#ok<AGROW>
        if ~prm.escape || n_esc >= prm.max_escapes, break; end
        n_esc = n_esc + 1;
        nsteps = prm.escape_steps * min(2^(n_esc - 1), prm.escape_growth_max);
        for k = 1:nsteps                      % random walk di fuga
            th = 2 * pi * rand;
            pn = p + prm.escape_step * [cos(th) sin(th)];
            if is_segment_free(p, pn, env, coll_step)
                p = pn; n = n + 1; path(n, :) = p;
            end
        end
        last_stall = n;
        continue;
    end
    h = min(prm.step, norm(p - goal));
    pn = p - h * g / ng;
    tries = 0;
    while ~is_segment_free(p, pn, env, coll_step) && tries < 6
        h = h / 2; pn = p - h * g / ng; tries = tries + 1;
    end
    if tries == 6, minima(end+1, :) = p; break; end %#ok<AGROW>
    p = pn; n = n + 1; path(n, :) = p;
end
raw = path(1:n, :);
info.raw_path = raw; info.local_minima = minima; info.escapes = n_esc;
info.iterations = it; info.goal = goal;
if ~success
    path = []; info.success = false;
    if isempty(minima)
        info.message = 'max_iter raggiunto';
    else
        info.message = sprintf('bloccato in un minimo locale in (%.2f, %.2f)', minima(end, 1), minima(end, 2));
    end
    return;
end
path = raw; info.success = true; info.message = 'ok';
end
