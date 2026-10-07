function out = run_project(p)
%RUN_PROJECT  Esegue l'intera pipeline: ambiente -> pianificazione -> controllo
%   -> simulazione -> analisi -> grafici/animazione.
%
%   out = run_project(p)   con p ottenuta da default_params() e personalizzata.
%   out.success, out.env, out.q0, out.qf, out.path, out.waypoints,
%   out.plan_info, out.sim, out.metrics
if nargin < 1, p = default_params(); end
[env, q0, qf] = build_environment(p);
validate_params(p, env, q0, qf);
out = struct('success', false, 'env', env, 'q0', q0, 'qf', qf, 'path', [], ...
             'waypoints', [], 'plan_info', [], 'sim', [], 'metrics', []);

% ---- 1) pianificazione
[path, waypoints, info] = plan_path(env, q0, qf, p);
out.plan_info = info; out.path = path; out.waypoints = waypoints;
if p.verbose
    fprintf('[%s] pianificazione: %s (%.3f s)\n', upper(info.planner), info.message, info.plan_time);
end
if p.viz.show_planner_debug, plot_planner_debug(env, q0, qf, path, info, p); end
if ~info.success
    warning('Pianificazione fallita: %s. Modificare i parametri del pianificatore.', info.message);
    return;
end

% ---- 2) controllo + simulazione con il modello differenziale
set_seed(p.seed);
sim = simulate_robot(waypoints, q0, qf, env, p);
out.sim = sim;

% ---- 3) analisi
m = compute_metrics(sim, path, waypoints, qf, env, info, p);
out.metrics = m;
out.success = sim.reached && ~m.collision;
if p.verbose
    print_metrics(m);
    if ~sim.reached, warning('Goal non raggiunto entro t_max = %.1f s.', p.sim.t_max); end
end

% ---- 4) grafici e animazione
if p.viz.show_plots, plot_results(sim, path, waypoints, q0, qf, env, m, p); end
if p.viz.animate,    animate_robot(sim, path, q0, qf, env, p); end
end
