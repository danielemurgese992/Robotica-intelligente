function plot_results_4R(res, p)
% PLOT_RESULTS_4R  Grafici dei risultati: q(t), e(t), tau(t) e percorso cartesiano.
t = res.t;
col = lines(4);

figure('Name', 'q(t): posizioni di giunto', 'Color', 'w');
for i = 1:4
    subplot(2,2,i);
    plot(t, rad2deg(res.qd(i,:)), '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(res.q(i,:)), '-', 'Color', col(i,:), 'LineWidth', 1.5);
    grid on; xlabel('t [s]'); ylabel(sprintf('\\theta_%d [deg]', i));
    title(sprintf('Giunto %d', i)); legend('riferimento q_d', 'reale q', 'Location', 'best');
end

figure('Name', 'e(t): errore di inseguimento', 'Color', 'w');
subplot(2,1,1);
plot(t, rad2deg(res.e), 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('e = q_d - q [deg]'); title('Errore nello spazio dei giunti');
legend('e_1', 'e_2', 'e_3', 'e_4', 'Location', 'best');
subplot(2,1,2);
semilogy(t, max(res.cartErr, 1e-12), 'k', 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('||p_d - p|| [m]'); title('Errore cartesiano dell''end-effector');

figure('Name', 'tau(t): coppie ai giunti', 'Color', 'w');
plot(t, res.tau, 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('\tau [N m]');
title(sprintf('Coppie ai giunti (controllore: %s)', strrep(p.ctrl.type, '_', ' ')));
legend('\tau_1', '\tau_2', '\tau_3', '\tau_4', 'Location', 'best');

figure('Name', 'Percorso cartesiano', 'Color', 'w');
plot(res.ref.X(1,:), res.ref.X(2,:), 'k--', 'LineWidth', 1.5); hold on;
plot(res.Xee(1,:), res.Xee(2,:), 'r-', 'LineWidth', 1.2);
plot(res.Xee(1,1), res.Xee(2,1), 'ro', 'MarkerFaceColor', 'r');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
legend('traiettoria desiderata', 'end-effector', 'posizione iniziale', 'Location', 'best');
title('Traiettoria nel piano XY');
end
