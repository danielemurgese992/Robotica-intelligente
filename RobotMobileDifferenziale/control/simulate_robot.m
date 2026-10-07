function res = simulate_robot(waypoints, q0, qf, env, p)
%SIMULATE_ROBOT  Inseguimento del percorso con il modello differenziale.
%
%   Ad ogni passo dt:
%     1) si sceglie il target (waypoint corrente o punto di riferimento continuo);
%     2) il controllore calcola (v, w) = (k_rho rho, k_alpha alpha);
%     3) (v, w) vengono limitati e convertiti in (omega_R, omega_L);
%     4) le ruote vengono saturate (preservando la curvatura) e, se richiesto,
%        disturbate con rumore gaussiano;
%     5) lo stato viene integrato con il modello cinematico differenziale.
%   Fasi: 1 = inseguimento, 2 = allineamento finale a theta_f, 3 = arrivato.
c = p.ctrl; rb = p.robot; dt = p.sim.dt;
N = ceil(p.sim.t_max / dt);
q = zeros(N + 1, 3); q(1, :) = q0;
[t, wR, wL, v, w, rho, alpha, phase_log] = deal(zeros(N + 1, 1));
target = zeros(N + 1, 2);
nwp = size(waypoints, 1);
s_cum = [0; cumsum(sqrt(sum(diff(waypoints, 1, 1).^2, 2)))];
L = s_cum(end);
idx = min(2, nwp); s_ref = 0; phase = 1;
goal = qf(1:2);
continuous = strcmpi(c.ref_mode, 'continuous');
k = 1;
for k = 1:N
    pos = q(k, 1:2);
    if phase == 1
        if continuous
            if norm(point_on_path(waypoints, s_cum, s_ref) - pos) < c.max_lag
                s_ref = min(s_ref + c.v_ref * dt, L);
            end
            tg = point_on_path(waypoints, s_cum, s_ref);
            at_end = s_ref >= L;
        else
            while idx < nwp && norm(waypoints(idx, :) - pos) < c.switch_radius
                idx = idx + 1;
            end
            tg = waypoints(idx, :);
            at_end = idx == nwp;
        end
        if at_end && norm(goal - pos) < c.goal_tol
            if c.align_final_heading, phase = 2; else, phase = 3; end
        end
    end
    if phase == 1
        [vc, wc, rho(k), alpha(k)] = controller_rho_alpha(q(k, :), tg, c);
    elseif phase == 2
        tg = goal;
        e = wrap_to_pi(qf(3) - q(k, 3));
        vc = 0; wc = c.k_theta * e;
        rho(k) = norm(goal - pos); alpha(k) = e;
        if abs(e) < c.heading_tol, phase = 3; end
    end
    target(k, :) = tg; t(k) = (k - 1) * dt; phase_log(k) = phase;
    if phase == 3
        rho(k) = norm(goal - pos); alpha(k) = wrap_to_pi(qf(3) - q(k, 3));
        break;
    end
    % limiti di velocita' del veicolo
    vc = min(max(vc, -rb.v_max), rb.v_max);
    wc = min(max(wc, -rb.w_max), rb.w_max);
    % (v, w) -> (omega_R, omega_L) + saturazione + rumore
    [r_, l_] = unicycle_to_wheels(vc, wc, rb);
    [r_, l_] = saturate_wheels(r_, l_, rb.wheel_max);
    if p.sim.wheel_noise_std > 0
        r_ = r_ + p.sim.wheel_noise_std * randn;
        l_ = l_ + p.sim.wheel_noise_std * randn;
    end
    wR(k) = r_; wL(k) = l_;
    [v(k), w(k)] = wheels_to_unicycle(r_, l_, rb);
    q(k + 1, :) = integrate_step(q(k, :), r_, l_, rb, dt, p.sim.integrator);
end
if phase ~= 3     % tempo massimo raggiunto: registra l'ultimo stato
    k = N + 1; t(k) = N * dt; target(k, :) = target(k - 1, :);
    rho(k) = norm(goal - q(k, 1:2)); phase_log(k) = phase;
end
K = k;
res.t = t(1:K); res.q = q(1:K, :);
res.wR = wR(1:K); res.wL = wL(1:K); res.v = v(1:K); res.w = w(1:K);
res.rho = rho(1:K); res.alpha = alpha(1:K); res.target = target(1:K, :);
res.phase = phase_log(1:K);
res.reached = phase == 3;
res.t_final = res.t(end);
end
