function [path, info] = plan_prm(env, q0, qf, prm, coll_step)
%PLAN_PRM  Probabilistic RoadMap.
%   1) Campiona n configurazioni uniformi e tiene quelle collision-free.
%   2) Collega ogni nodo ai k vicini entro connection_radius con archi liberi.
%   3) Cerca il cammino q0 -> qf (Dijkstra / A*).
%   Se q0 e qf restano in componenti diverse, ripete con piu' campioni.
b = env.bounds; m = env.inflation;
for attempt = 1:prm.max_attempts
    n = round(prm.n_samples * prm.growth^(attempt - 1));
    S = [b(1) + m + rand(n, 1) * (b(2) - b(1) - 2*m), ...
         b(3) + m + rand(n, 1) * (b(4) - b(3) - 2*m)];
    S = S(is_free(S, env), :);
    V = [q0(1:2); qf(1:2); S];
    N = size(V, 1);
    nbrs = cell(N, 1); costs = cell(N, 1);
    tested = false(N);
    E = zeros(0, 2);
    for i = 1:N
        d = sqrt(sum((V - V(i, :)).^2, 2)); d(i) = inf;
        [ds, ord] = sort(d);
        ord = ord(ds <= prm.connection_radius);
        ord = ord(1:min(prm.k_neighbors, numel(ord)));
        for j = ord'
            if tested(i, j), continue; end
            tested(i, j) = true; tested(j, i) = true;
            if is_segment_free(V(i, :), V(j, :), env, coll_step)
                nbrs{i}(end+1) = j; costs{i}(end+1) = d(j);
                nbrs{j}(end+1) = i; costs{j}(end+1) = d(j);
                E(end+1, :) = [i j]; %#ok<AGROW>
            end
        end
    end
    nodes = graph_search(nbrs, costs, 1, 2, prm.search, V);
    if ~isempty(nodes), break; end
end
info.nodes = V; info.edges = E; info.attempts = attempt;
info.n_nodes = N; info.n_edges = size(E, 1);
if isempty(nodes)
    path = []; info.success = false;
    info.message = 'q0 e qf in componenti non connesse della roadmap'; return;
end
path = V(nodes, :); info.success = true; info.message = 'ok';
end
