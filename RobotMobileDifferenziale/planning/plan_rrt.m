function [path, info] = plan_rrt(env, q0, qf, prm, coll_step)
%PLAN_RRT  Rapidly-exploring Random Tree.
%   Ad ogni iterazione campiona un punto (il goal con probabilita' goal_bias),
%   trova il nodo piu' vicino dell'albero, avanza di 'step' verso il campione
%   e aggiunge il nuovo nodo se il segmento e' libero. Termina quando un nodo
%   entro goal_tol vede il goal.
b = env.bounds; m = env.inflation; goal = qf(1:2);
Nmax = prm.max_iter + 2;
V = zeros(Nmax, 2); parent = zeros(Nmax, 1);
V(1, :) = q0(1:2); n = 1; success = false;
for it = 1:prm.max_iter
    if rand < prm.goal_bias
        xr = goal;
    else
        xr = [b(1) + m + rand * (b(2) - b(1) - 2*m), b(3) + m + rand * (b(4) - b(3) - 2*m)];
    end
    [~, in] = min(sum((V(1:n, :) - xr).^2, 2));
    dir = xr - V(in, :); L = norm(dir);
    if L < 1e-9, continue; end
    xn = V(in, :) + dir / L * min(prm.step, L);
    if ~is_segment_free(V(in, :), xn, env, coll_step), continue; end
    n = n + 1; V(n, :) = xn; parent(n) = in;
    if norm(xn - goal) <= prm.goal_tol && is_segment_free(xn, goal, env, coll_step)
        n = n + 1; V(n, :) = goal; parent(n) = n - 1;
        success = true; break;
    end
end
V = V(1:n, :); parent = parent(1:n);
info.nodes = V; info.parent = parent; info.iterations = it; info.n_nodes = n;
if ~success
    path = []; info.success = false;
    info.message = 'goal non raggiunto entro max_iter'; return;
end
idx = n;
while parent(idx(1)) ~= 0, idx = [parent(idx(1)); idx]; end %#ok<AGROW>
path = V(idx, :); info.success = true; info.message = 'ok';
end
