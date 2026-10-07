%% MAIN_4R  Robot manipolatore planare 4R con gravita' - script principale
%
%  Esegue in sequenza (attivabili con i flag p.run.*):
%    1. cinematica diretta: tabella DH, matrici A_i, T, grafico del robot
%    2. cinematica inversa: soluzioni gomito alto/basso e verifica FK(IK(p))
%    3. dinamica: M, C, G nella configurazione di prova
%    4. simulazione ode45 del robot in anello chiuso lungo la traiettoria
%    5. grafici q(t), e(t), tau(t)
%    6. animazione con drawnow (terne DH e target)
%
%  Tutti i valori numerici sono in params_4R.m: modificarli li' oppure
%  sovrascriverli nella sezione "PERSONALIZZAZIONE" qui sotto.

clear; close all; clc;
p = params_4R();

%% ===================== PERSONALIZZAZIONE (opzionale) ===================
% Esempi (decommentare per provare):
% p.robot.L      = [0.5 0.4 0.3 0.15];
% p.traj.type    = 'line';
% p.ctrl.type    = 'pd_gravity';     % PD + compensazione di gravita' (guadagni in p.ctrl.pd)
% p.ctrl.modelScale = 1.2;          % modello del controllore errato del +20%
% p.ctrl.Ki     = 200*eye(4);       % azione integrale (recupera l'errore di modello)
% p.ik.elbow     = 'down';
% p.run.animate  = false;

L = p.robot.L;

%% 1. CINEMATICA DIRETTA
if p.run.fkDemo
    q = p.viz.qDemo(:);
    fprintf('=== CINEMATICA DIRETTA ===\n');
    DH = dh_table_4R(q, L);
    fprintf('Tabella DH  [a_i  alpha_i  d_i  theta_i(deg)]:\n');
    disp([DH(:,1:3) rad2deg(DH(:,4))]);
    for i = 1:4
        fprintf('A_%d =\n', i); disp(dh_matrix(DH(i,1), DH(i,2), DH(i,3), DH(i,4)));
    end
    [T, x, y, phi] = fk_4R(q, L);
    fprintf('T = A1*A2*A3*A4 =\n'); disp(T);
    fprintf('x = %.4f m,  y = %.4f m,  phi = %.2f deg\n\n', x, y, rad2deg(phi));

    figure('Name', 'Cinematica diretta', 'Color', 'w'); ax = gca;
    plot_robot_4R(ax, q, L, [], p.viz.frameScale);
    R = sum(L)*1.05; axis(ax, 'equal'); axis(ax, [-R R -R R]); grid(ax, 'on');
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('q = [%s] deg  ->  (x, y, \\phi) = (%.3f, %.3f, %.1f°)', ...
        num2str(rad2deg(q.'), '%.0f '), x, y, rad2deg(phi)));
end

%% 2. CINEMATICA INVERSA
if p.run.ikDemo
    fprintf('=== CINEMATICA INVERSA ===\n');
    [~, x, y, phi] = fk_4R(p.viz.qDemo, L);       % punto di prova raggiungibile
    psi = phi + p.ik.psiOffset;
    [~, info] = ik_4R(x, y, phi, L, 'up', psi);
    fprintf('Target (x, y, phi) = (%.4f, %.4f, %.2f deg), psi = %.2f deg\n', ...
        x, y, rad2deg(phi), rad2deg(psi));
    fprintf('Polso (xw, yw) = (%.4f, %.4f)\n', info.wrist);
    names = {'gomito alto ', 'gomito basso'};
    figure('Name', 'Cinematica inversa', 'Color', 'w');
    for k = 1:2
        qk = info.solutions(:,k);
        [~, xk, yk, phik] = fk_4R(qk, L);
        fprintf('%s: theta = [%s] deg | errore FK(IK) = %.2e\n', names{k}, ...
            num2str(rad2deg(qk.'), '%8.2f'), norm([xk-x, yk-y, mod(phik-phi+pi,2*pi)-pi]));
        ax = subplot(1,2,k);
        plot_robot_4R(ax, qk, L, [], p.viz.frameScale);
        plot(ax, x, y, 'go', 'MarkerSize', 14, 'LineWidth', 2);
        R = sum(L)*1.05; axis(ax, 'equal'); axis(ax, [-R R -R R]); grid(ax, 'on');
        title(ax, strtrim(names{k}));
    end
    fprintf('\n');
end

%% 3. DINAMICA (valori nella configurazione di prova)
fprintf('=== DINAMICA (q = qDemo, dq = 0.5 rad/s) ===\n');
[M, C, G] = dynamics_4R(p.viz.qDemo, 0.5*ones(4,1), p.robot);
fprintf('M(q) =\n'); disp(M);
fprintf('C(q,dq) =\n'); disp(C);
fprintf('G(q) =\n'); disp(G);

%% 4-6. SIMULAZIONE, GRAFICI E ANIMAZIONE
if p.run.simulate
    fprintf('=== SIMULAZIONE (%s, traiettoria %s) ===\n', p.ctrl.type, p.traj.type);
    res = simulate_4R(p);
    if p.run.plots,   plot_results_4R(res, p); end
    if p.run.animate, animate_4R(res, p);      end
end
