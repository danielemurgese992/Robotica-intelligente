function W = resample_path(P, spacing)
%RESAMPLE_PATH  Ricampiona la polilinea P con waypoint a distanza <= spacing.
%   Ogni segmento e' suddiviso in ceil(L/spacing) parti uguali e i vertici
%   originali sono SEMPRE conservati: il percorso ricampionato coincide con
%   quello pianificato (nessun "taglio" degli spigoli), quindi resta
%   collision-free.
seg = sqrt(sum(diff(P, 1, 1).^2, 2));
P = P([true; seg > 1e-9], :);
if size(P, 1) < 2, W = P; return; end
W = P(1, :);
for i = 1:size(P, 1) - 1
    a = P(i, :); b = P(i + 1, :);
    n = max(1, ceil(norm(b - a) / spacing));
    t = (1:n)' / n;
    W = [W; a + t * (b - a)]; %#ok<AGROW>
end
end
