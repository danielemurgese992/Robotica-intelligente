function P = shortcut_path(P, env, step)
%SHORTCUT_PATH  Semplificazione "string pulling" di un percorso collision-free.
%   Da ogni punto i salta al punto j piu' lontano raggiungibile in linea retta
%   (scansione in avanti): elimina zig-zag di PRM/RRT/APF e i punti allineati
%   della griglia, mantenendo il percorso libero da collisioni.
if size(P, 1) <= 2, return; end
keep = 1; i = 1; n = size(P, 1);
while i < n
    j = i + 1;
    while j < n && is_segment_free(P(i, :), P(j + 1, :), env, step)
        j = j + 1;
    end
    keep(end+1) = j; %#ok<AGROW>
    i = j;
end
P = P(keep, :);
end
