function tf = is_path_free(P, env, step)
%IS_PATH_FREE  Verifica che tutti i segmenti della polilinea P siano liberi.
tf = true;
for i = 1:size(P, 1) - 1
    if ~is_segment_free(P(i, :), P(i+1, :), env, step)
        tf = false; return;
    end
end
end
