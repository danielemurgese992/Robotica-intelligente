function ok = run_all_tests()
%RUN_ALL_TESTS  Verifiche automatiche di modello, geometria, pianificatori e controllo.
%   >> addpath(genpath(pwd)); run_all_tests
%   Restituisce true se tutti i test passano.
p = default_params();
p.viz.show_plots = false; p.viz.animate = false;
p.viz.show_planner_debug = false; p.verbose = false;
rb = p.robot;
n_ok = 0; n_tot = 0;

    function check(name, cond)
        n_tot = n_tot + 1;
        if cond, n_ok = n_ok + 1; fprintf('  [PASS] %s\n', name);
        else, fprintf('  [FAIL] %s\n', name); end
    end

fprintf('=== Modello cinematico ===\n');
[wR, wL] = unicycle_to_wheels(0.37, -1.1, rb);
[v, w] = wheels_to_unicycle(wR, wL, rb);
check('conversione (v,w) <-> (wR,wL) invertibile', abs(v - 0.37) < 1e-12 && abs(w + 1.1) < 1e-12);

q = [0 0 0]; om = 5; T = 2; dt = 0.01;
for k = 1:round(T / dt), q = integrate_step(q, om, om, rb, dt, 'rk4'); end
check('ruote uguali -> moto rettilineo, x = r*omega*t', abs(q(1) - rb.r*om*T) < 1e-9 && abs(q(2)) < 1e-12 && abs(q(3)) < 1e-12);

q = [1 2 0]; T = 1;
for k = 1:round(T / dt), q = integrate_step(q, om, -om, rb, dt, 'rk4'); end
check('ruote opposte -> rotazione pura, theta = 2 r omega t / d', ...
      norm(q(1:2) - [1 2]) < 1e-12 && abs(q(3) - wrap_to_pi(2*rb.r*om*T/rb.d)) < 1e-9);

qe = integrate_step([0.3 -0.2 0.4], 12, 7, rb, 0.05, 'exact');
qr = integrate_step([0.3 -0.2 0.4], 12, 7, rb, 0.05, 'rk4');
check('RK4 coincide con la soluzione esatta (errore < 1e-8)', norm(qe - qr) < 1e-8);

[a, b, s] = saturate_wheels(40, 10, 20);
check('saturazione: limite rispettato e curvatura preservata', abs(a - 20) < 1e-12 && abs(b - 5) < 1e-12 && s == 0.5);

fprintf('=== Geometria ===\n');
env.bounds = [0 10 0 10]; env.inflation = 0.25; env.robot_radius = 0.2;
env.obstacles = {make_obstacle('circle', [5 5], 1), make_obstacle('polygon', [1 1; 2 1; 2 2; 1 2])};
D = obstacle_distances([7 5; 1.5 1.5; 3 1.5], env);
check('distanza da un cerchio', abs(D(1, 1) - 1) < 1e-12);
check('distanza negativa dentro un poligono', abs(D(2, 2) + 0.5) < 1e-12);
check('distanza esterna da un poligono', abs(D(3, 2) - 1) < 1e-12);
check('segmento che attraversa un ostacolo non e'' libero', ~is_segment_free([3 5], [7 5], env, 0.02));
check('segmento lontano dagli ostacoli e'' libero', is_segment_free([3 8], [7 8], env, 0.02));

fprintf('=== Pianificatori (scenario base) ===\n');
for pl = {'cell', 'prm', 'rrt', 'apf'}
    pp = p; pp.planner = pl{1};
    [env2, q0, qf] = build_environment(pp);
    [path, wp, info] = plan_path(env2, q0, qf, pp);
    good = info.success && is_path_free(path, env2, 0.01) && is_path_free(wp, env2, 0.01) ...
           && norm(path(1, :) - q0(1:2)) < 1e-9 && norm(path(end, :) - qf(1:2)) < 1e-9;
    check(sprintf('%s: percorso q0 -> qf collision-free', upper(pl{1})), good);
end

fprintf('=== Minimi locali APF (scenario trap) ===\n');
pp = p; pp.planner = 'apf'; pp.env.scenario = 'trap'; pp.apf.escape = false;
[env3, q0, qf] = build_environment(pp);
[~, ~, info] = plan_path(env3, q0, qf, pp);
check('APF senza fuga si blocca nel minimo locale della U', ~info.success && ~isempty(info.local_minima));
pp.apf.escape = true;
[path, ~, info] = plan_path(env3, q0, qf, pp);
check('APF con random walk esce dal minimo locale', info.success && is_path_free(path, env3, 0.01));

fprintf('=== Controllo e simulazione ===\n');
for mode = {'waypoints', 'continuous'}
    pp = p; pp.ctrl.ref_mode = mode{1};
    out = run_project(pp);
    m = out.metrics;
    check(sprintf('controllo %s: goal raggiunto senza collisioni', mode{1}), ...
          out.success && m.final_pos_error < pp.ctrl.goal_tol && ...
          m.final_head_error < pp.ctrl.heading_tol && m.max_wheel <= rb.wheel_max + 1e-9);
end

fprintf('\n%d / %d test superati\n', n_ok, n_tot);
ok = n_ok == n_tot;
end
