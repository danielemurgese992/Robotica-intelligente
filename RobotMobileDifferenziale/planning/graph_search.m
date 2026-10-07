function [nodes, cost] = graph_search(nbrs, costs, s, t, method, coords)
%GRAPH_SEARCH  Cammino minimo su grafo pesato (Dijkstra o A*).
%
%   [nodes, cost] = graph_search(nbrs, costs, s, t, method, coords)
%   nbrs{i}  : indici dei vicini del nodo i
%   costs{i} : costi (>= 0) dei rispettivi archi
%   s, t     : nodo di partenza e di arrivo
%   method   : 'dijkstra' (h = 0) oppure 'astar' (h = distanza euclidea)
%   coords   : N x 2 coordinate (usate solo da A*)
%   nodes    : sequenza di nodi da s a t ([] se t non raggiungibile)
N = numel(nbrs);
g = inf(N, 1); g(s) = 0;
prev = zeros(N, 1);
closed = false(N, 1);
h = zeros(N, 1);
if strcmpi(method, 'astar')
    h = sqrt(sum((coords - coords(t, :)).^2, 2));
end

while true
    f = g + h; f(closed) = inf;
    [fmin, u] = min(f);
    if isinf(fmin) || u == t, break; end
    closed(u) = true;
    nb = nbrs{u}(:); c = costs{u}(:);
    if isempty(nb), continue; end
    alt = g(u) + c;
    upd = alt < g(nb) & ~closed(nb);
    g(nb(upd)) = alt(upd);
    prev(nb(upd)) = u;
end

cost = g(t);
if isinf(cost), nodes = []; return; end
nodes = t;
while nodes(1) ~= s
    nodes = [prev(nodes(1)); nodes]; %#ok<AGROW>
end
end
