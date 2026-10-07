function m = compute_metrics(res, path, waypoints, qf, env, info, p)
%COMPUTE_METRICS  Indicatori quantitativi di pianificazione e controllo.
Q = res.q(:, 1:2);
ct = point_to_path_distance(Q, waypoints);
m.planner          = info.planner;
m.plan_time        = info.plan_time;
m.planned_length   = path_length(path);
m.executed_length  = path_length(Q);
m.reached          = res.reached;
m.time_to_goal     = res.t_final;
m.final_pos_error  = norm(Q(end, :) - qf(1:2));
m.final_head_error = abs(wrap_to_pi(res.q(end, 3) - qf(3)));
m.cross_track      = ct;
m.ct_rms           = sqrt(mean(ct.^2));
m.ct_max           = max(ct);
cl_plan = clearance(resample_path(path, 0.02), env) - env.robot_radius;
cl_exec = clearance(Q, env) - env.robot_radius;
m.min_clear_plan   = min(cl_plan);      % distanza minima bordo-robot / ostacolo
m.min_clear_exec   = min(cl_exec);
m.collision        = any(cl_exec <= 0);
m.max_wheel        = max(max(abs(res.wR)), max(abs(res.wL)));
m.saturated_frac   = mean(max(abs(res.wR), abs(res.wL)) >= p.robot.wheel_max - 1e-9);
m.wheel_effort     = sum((res.wR.^2 + res.wL.^2)) * p.sim.dt;
m.path_efficiency  = m.planned_length / max(m.executed_length, eps);
end
