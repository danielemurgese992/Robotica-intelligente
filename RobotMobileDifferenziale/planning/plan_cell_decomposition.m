function [path, info] = plan_cell_decomposition(env, q0, qf, prm)
%PLAN_CELL_DECOMPOSITION  Decomposizione in celle + retraction + ricerca su grafo.
%
%   1) Il rettangolo di lavoro e' diviso in celle quadrate di lato prm.resolution.
%   2) Una cella e' LIBERA se la sua clearance supera il raggio gonfiato del
%      robot (+ mezza diagonale della cella se prm.conservative = true).
%   3) Si costruisce il grafo di adiacenza delle celle libere (4 o 8 vicini;
%      con 8 vicini le diagonali sono ammesse solo senza "tagliare l'angolo").
%   4) Retraction: ogni cella libera e' ridotta al suo nodo centrale; il costo
%      di un arco e' la lunghezza moltiplicata per (1 + w/clearance) cosi' che
%      il cammino sia "ritratto" verso il centro dello spazio libero.
%   5) Dijkstra (o A*) dal nodo-cella di q0 a quello di qf.
res = prm.resolution; b = env.bounds;
nx = floor((b(2) - b(1)) / res); ny = floor((b(4) - b(3)) / res);
xc = b(1) + ((1:nx) - 0.5) * res;
yc = b(3) + ((1:ny) - 0.5) * res;
[X, Y] = meshgrid(xc, yc);
C = [X(:) Y(:)];
cl = clearance(C, env);
thr = env.inflation;
if prm.conservative, thr = thr + res * sqrt(2) / 2; end
free = cl > thr;
F = reshape(free, ny, nx);

if prm.connectivity == 4
    off = [1 0; -1 0; 0 1; 0 -1];
else
    off = [1 0; -1 0; 0 1; 0 -1; 1 1; 1 -1; -1 1; -1 -1];
end
w = prm.clearance_weight;
Ncell = nx * ny;
nbrs = cell(Ncell, 1); costs = cell(Ncell, 1);
ids = find(free);
for k = 1:numel(ids)
    i = ids(k);
    iy = mod(i - 1, ny) + 1; ix = floor((i - 1) / ny) + 1;
    nb = zeros(1, 8); cc = zeros(1, 8); m = 0;
    for o = 1:size(off, 1)
        jy = iy + off(o, 1); jx = ix + off(o, 2);
        if jy < 1 || jy > ny || jx < 1 || jx > nx || ~F(jy, jx), continue; end
        if off(o, 1) ~= 0 && off(o, 2) ~= 0 && (~F(jy, ix) || ~F(iy, jx)), continue; end
        j = jy + (jx - 1) * ny;
        L = res * norm(off(o, :));
        cavg = max(0.5 * (cl(i) + cl(j)) - env.inflation, 1e-3);
        m = m + 1; nb(m) = j; cc(m) = L * (1 + w / cavg);
    end
    nbrs{i} = nb(1:m); costs{i} = cc(1:m);
end

s = nearest_visible_cell(q0(1:2), C, ids, env);
t = nearest_visible_cell(qf(1:2), C, ids, env);
info.grid = struct('xc', xc, 'yc', yc, 'free', F, 'res', res);
info.n_free = numel(ids); info.n_cells = Ncell;
if isempty(s) || isempty(t)
    path = []; info.success = false;
    info.message = 'q0 o qf non collegabili a una cella libera'; return;
end
[nodes, cost] = graph_search(nbrs, costs, s, t, prm.search, C);
if isempty(nodes)
    path = []; info.success = false;
    info.message = 'nessun cammino nel grafo di adiacenza'; return;
end
info.retracted_nodes = C(nodes, :);
info.cost = cost; info.success = true; info.message = 'ok';
path = [q0(1:2); C(nodes, :); qf(1:2)];
end

function c = nearest_visible_cell(p, C, ids, env)
% cella libera piu' vicina raggiungibile in linea retta da p
d = sqrt(sum((C(ids, :) - p).^2, 2));
[~, ord] = sort(d);
c = [];
for k = 1:min(50, numel(ord))
    cand = ids(ord(k));
    if is_segment_free(p, C(cand, :), env, 0.02)
        c = cand; return;
    end
end
end
