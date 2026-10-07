function T = compare_planners(p, planners, n_runs)
%COMPARE_PLANNERS  Confronta i pianificatori sullo stesso scenario.
%
%   T = compare_planners(p)                       tutti i metodi, 1 esecuzione
%   T = compare_planners(p, {'prm','rrt'}, 10)    10 esecuzioni (semi diversi)
%
%   Per i metodi stocastici (PRM, RRT, APF con fuga) si usano i semi
%   p.seed, p.seed+1, ...; le metriche riportate sono medie sulle esecuzioni
%   riuscite. T e' una struct array con una riga per metodo.
if nargin < 2 || isempty(planners), planners = {'cell', 'prm', 'rrt', 'apf'}; end
if nargin < 3, n_runs = 1; end
p.viz.show_plots = false; p.viz.animate = false;
p.viz.show_planner_debug = false; p.verbose = false;
T = struct([]);
for i = 1:numel(planners)
    acc = []; ok = 0;
    for r = 1:n_runs
        pp = p; pp.planner = planners{i}; pp.seed = p.seed + r - 1;
        out = run_project(pp);
        if out.success
            ok = ok + 1;
            m = out.metrics;
            row = [m.plan_time m.planned_length m.time_to_goal m.ct_rms m.ct_max ...
                   m.min_clear_exec m.final_pos_error m.max_wheel];
            acc = [acc; row]; %#ok<AGROW>
        end
    end
    T(i).planner = planners{i};
    T(i).success_rate = ok / n_runs;
    if ok > 0
        mu = mean(acc, 1);
    else
        mu = nan(1, 8);
    end
    T(i).plan_time = mu(1); T(i).length = mu(2); T(i).time_to_goal = mu(3);
    T(i).ct_rms = mu(4); T(i).ct_max = mu(5); T(i).min_clear = mu(6);
    T(i).final_err = mu(7); T(i).max_wheel = mu(8);
end
fprintf('\nConfronto pianificatori - scenario "%s" (%d esecuzioni per metodo)\n', p.env.scenario, n_runs);
fprintf('%-6s %7s %9s %9s %9s %9s %9s %9s %10s\n', 'Metodo', 'Succ.', 't_plan[s]', ...
        'L[m]', 't_goal[s]', 'eRMS[m]', 'eMax[m]', 'clr[m]', 'wMax[r/s]');
for i = 1:numel(T)
    fprintf('%-6s %6.0f%% %9.3f %9.2f %9.2f %9.4f %9.4f %9.3f %10.2f\n', upper(T(i).planner), ...
        100 * T(i).success_rate, T(i).plan_time, T(i).length, T(i).time_to_goal, ...
        T(i).ct_rms, T(i).ct_max, T(i).min_clear, T(i).max_wheel);
end
end
