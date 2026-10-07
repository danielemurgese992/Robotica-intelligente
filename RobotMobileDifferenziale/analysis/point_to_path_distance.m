function d = point_to_path_distance(Q, P)
%POINT_TO_PATH_DISTANCE  Distanza minima di ogni punto Q(i,:) dalla polilinea P.
%   E' l'errore di tracking laterale (cross-track error).
n = size(Q, 1); d = inf(n, 1);
if size(P, 1) == 1
    d = sqrt(sum((Q - P).^2, 2)); return;
end
for k = 1:size(P, 1) - 1
    a = P(k, :); ab = P(k + 1, :) - a; L2 = max(sum(ab.^2), 1e-12);
    tt = min(max(((Q(:, 1) - a(1)) * ab(1) + (Q(:, 2) - a(2)) * ab(2)) / L2, 0), 1);
    dk = sqrt((Q(:, 1) - a(1) - tt * ab(1)).^2 + (Q(:, 2) - a(2) - tt * ab(2)).^2);
    d = min(d, dk);
end
end
