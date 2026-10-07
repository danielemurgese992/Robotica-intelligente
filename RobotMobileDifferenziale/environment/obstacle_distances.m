function [D, Gx, Gy] = obstacle_distances(P, env)
%OBSTACLE_DISTANCES  Distanza con segno da ogni ostacolo e da ogni bordo.
%
%   [D, Gx, Gy] = obstacle_distances(P, env)
%   P   : N x 2 punti del piano
%   D   : N x (M+4) distanze con segno (negativa = punto dentro l'ostacolo);
%         le ultime 4 colonne sono i bordi xmin, xmax, ymin, ymax
%   Gx,Gy: componenti del gradiente della distanza (versore che si allontana
%         dall'ostacolo), usate dai campi di potenziale repulsivi.
N = size(P, 1);
M = numel(env.obstacles);
D = zeros(N, M + 4); Gx = D; Gy = D;
x = P(:, 1); y = P(:, 2);

for j = 1:M
    o = env.obstacles{j};
    if strcmp(o.type, 'circle')
        vx = x - o.center(1); vy = y - o.center(2);
        n = sqrt(vx.^2 + vy.^2);
        D(:, j) = n - o.radius;
        n(n < 1e-12) = 1e-12;
        Gx(:, j) = vx ./ n; Gy(:, j) = vy ./ n;
    else
        V = o.vertices; nv = size(V, 1);
        dmin = inf(N, 1); cx = zeros(N, 1); cy = zeros(N, 1);
        for k = 1:nv
            a = V(k, :); b = V(mod(k, nv) + 1, :);
            ab = b - a; L2 = sum(ab.^2);
            t = ((x - a(1)) * ab(1) + (y - a(2)) * ab(2)) / L2;
            t = min(max(t, 0), 1);
            px = a(1) + t * ab(1); py = a(2) + t * ab(2);
            dk = sqrt((x - px).^2 + (y - py).^2);
            better = dk < dmin;
            dmin(better) = dk(better); cx(better) = px(better); cy(better) = py(better);
        end
        inside = inpolygon(x, y, V(:, 1), V(:, 2));
        sgn = ones(N, 1); sgn(inside) = -1;
        D(:, j) = sgn .* dmin;
        dd = dmin; dd(dd < 1e-12) = 1e-12;
        Gx(:, j) = sgn .* (x - cx) ./ dd;
        Gy(:, j) = sgn .* (y - cy) ./ dd;
    end
end

b = env.bounds;
D(:, M+1) = x - b(1);  Gx(:, M+1) =  1;
D(:, M+2) = b(2) - x;  Gx(:, M+2) = -1;
D(:, M+3) = y - b(3);  Gy(:, M+3) =  1;
D(:, M+4) = b(4) - y;  Gy(:, M+4) = -1;
end
