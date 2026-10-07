function figs = plot_results(res, path, waypoints, q0, qf, env, m, p)
%PLOT_RESULTS  Grafici richiesti: traiettoria pianificata ed eseguita, errore
%   di tracking, velocita' delle ruote (e v, w), evoluzione dello stato.
figs = [];
% 1) traiettoria pianificata vs eseguita
figs(end+1) = figure('Name', 'Traiettorie', 'Color', 'w'); ax = gca;
plot_environment(env, ax);
h1 = plot(ax, path(:, 1), path(:, 2), 'b--', 'LineWidth', 1.5);
h2 = plot(ax, waypoints(:, 1), waypoints(:, 2), 'b.', 'MarkerSize', 10);
h3 = plot(ax, res.q(:, 1), res.q(:, 2), 'r-', 'LineWidth', 2);
plot_pose(ax, q0, [0 0.7 0], 'q_0'); plot_pose(ax, qf, [0.85 0 0], 'q_f');
legend([h1 h2 h3], {'pianificata', 'waypoint', 'eseguita'}, 'Location', 'southeast');
title(ax, sprintf('%s - traiettoria pianificata ed eseguita', upper(m.planner)));

% 2) errore di tracking
figs(end+1) = figure('Name', 'Errore di tracking', 'Color', 'w');
subplot(2, 1, 1);
plot(res.t, m.cross_track, 'LineWidth', 1.5); grid on;
ylabel('e_{ct} [m]');
title(sprintf('Errore laterale dal percorso (RMS %.3f m, max %.3f m)', m.ct_rms, m.ct_max));
subplot(2, 1, 2);
plot(res.t, res.rho, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('\rho [m]');
title('Distanza dal target corrente \rho');

% 3) velocita' delle ruote e del veicolo
figs(end+1) = figure('Name', 'Velocita'' ruote', 'Color', 'w');
subplot(2, 1, 1); hold on; grid on;
plot(res.t, res.wR, 'r', 'LineWidth', 1.5); plot(res.t, res.wL, 'b', 'LineWidth', 1.5);
plot(res.t([1 end]), p.robot.wheel_max * [1 1], 'k--');
plot(res.t([1 end]), -p.robot.wheel_max * [1 1], 'k--');
ylabel('[rad/s]'); legend('\omega_R', '\omega_L', 'limite'); title('Velocita'' angolari delle ruote');
subplot(2, 1, 2); hold on; grid on;
plot(res.t, res.v, 'LineWidth', 1.5); plot(res.t, res.w, 'LineWidth', 1.5);
xlabel('t [s]'); legend('v [m/s]', '\omega [rad/s]'); title('Velocita'' lineare e angolare');

% 4) stato
figs(end+1) = figure('Name', 'Stato', 'Color', 'w');
lab = {'x [m]', 'y [m]', '\theta [rad]'};
for i = 1:3
    subplot(3, 1, i);
    s = res.q(:, i); if i == 3, s = unwrap(s); end
    plot(res.t, s, 'LineWidth', 1.5); grid on; ylabel(lab{i});
end
xlabel('t [s]');

if p.viz.save_figures
    if ~exist(p.viz.output_dir, 'dir'), mkdir(p.viz.output_dir); end
    names = {'traiettorie', 'errore_tracking', 'velocita_ruote', 'stato'};
    for i = 1:numel(figs)
        print(figs(i), fullfile(p.viz.output_dir, sprintf('%s_%s.png', m.planner, names{i})), '-dpng', '-r120');
    end
end
end
