function [path, waypoints, info] = plan_path(env, q0, qf, p)
%PLAN_PATH  Esegue il pianificatore scelto e post-elabora il percorso.
%   path      : percorso geometrico pianificato (dopo shortcut se attivo)
%   waypoints : percorso ricampionato a passo p.post.waypoint_spacing
%   info      : diagnostica del pianificatore (+ tempo, lunghezze)
set_seed(p.seed);
cs = p.post.collision_step;
t0 = tic;
switch lower(p.planner)
    case 'cell', [raw, info] = plan_cell_decomposition(env, q0, qf, p.cell);
    case 'prm',  [raw, info] = plan_prm(env, q0, qf, p.prm, cs);
    case 'rrt',  [raw, info] = plan_rrt(env, q0, qf, p.rrt, cs);
    case 'apf',  [raw, info] = plan_apf(env, q0, qf, p.apf, cs);
end
info.planner = lower(p.planner);
if ~info.success
    info.plan_time = toc(t0);
    path = []; waypoints = [];
    return;
end
info.raw = raw;
info.raw_length = path_length(raw);
path = raw;
if p.post.shortcut, path = shortcut_path(raw, env, cs); end
waypoints = resample_path(path, p.post.waypoint_spacing);
info.plan_time = toc(t0);
info.length = path_length(path);
end
