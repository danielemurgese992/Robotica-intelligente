function validate_params(p, env, q0, qf)
%VALIDATE_PARAMS  Controlla la coerenza dei parametri prima dell'esecuzione.
%   Lancia un errore esplicito invece di lasciar fallire il codice a valle.

chk = {p.robot.r, 'robot.r'; p.robot.d, 'robot.d'; p.robot.radius, 'robot.radius';
       p.robot.wheel_max, 'robot.wheel_max'; p.robot.v_max, 'robot.v_max';
       p.robot.w_max, 'robot.w_max'; p.ctrl.k_rho, 'ctrl.k_rho';
       p.ctrl.k_alpha, 'ctrl.k_alpha'; p.sim.dt, 'sim.dt'; p.sim.t_max, 'sim.t_max';
       p.post.waypoint_spacing, 'post.waypoint_spacing'};
for i = 1:size(chk, 1)
    v = chk{i, 1};
    if ~(isnumeric(v) && isscalar(v) && v > 0)
        error('Il parametro p.%s deve essere uno scalare > 0.', chk{i, 2});
    end
end

if ~any(strcmpi(p.planner, {'cell','prm','rrt','apf'}))
    error('p.planner deve essere cell | prm | rrt | apf.');
end
if ~any(strcmpi(p.ctrl.ref_mode, {'waypoints','continuous'}))
    error('p.ctrl.ref_mode deve essere waypoints | continuous.');
end
if ~any(strcmpi(p.sim.integrator, {'euler','rk4','exact'}))
    error('p.sim.integrator deve essere euler | rk4 | exact.');
end
if ~any(p.cell.connectivity == [4 8])
    error('p.cell.connectivity deve essere 4 oppure 8.');
end
if p.ctrl.k_alpha <= p.ctrl.k_rho
    warning('Si consiglia k_alpha > k_rho per la convergenza del controllore.');
end

if numel(q0) ~= 3 || numel(qf) ~= 3
    error('q0 e qf devono essere vettori [x y theta].');
end
if ~is_free(q0(1:2), env)
    error('q0 = %s e'' in collisione o troppo vicina a un ostacolo/bordo.', mat2str(q0, 3));
end
if ~is_free(qf(1:2), env)
    error('qf = %s e'' in collisione o troppo vicina a un ostacolo/bordo.', mat2str(qf, 3));
end
end
