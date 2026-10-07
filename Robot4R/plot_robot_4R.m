function h = plot_robot_4R(ax, theta, L, h, frameScale)
% PLOT_ROBOT_4R  Disegna il robot 4R e le terne DH di ogni giunto.
%
%   h = plot_robot_4R(ax, theta, L)                  primo disegno
%   h = plot_robot_4R(ax, theta, L, h)               aggiorna un disegno esistente
%   h = plot_robot_4R(ax, theta, L, h, frameScale)   lunghezza assi terne [m]
%
%   Le terne DH 0..4 sono disegnate con asse x in rosso e asse y in verde;
%   l'asse z (uscente dal piano) e' indicato dal pallino sull'origine.
%   Restituisce la struttura di handle grafici (per animazioni veloci).
if nargin < 4, h = []; end
if nargin < 5 || isempty(frameScale), frameScale = 0.15*sum(L)/2.5; end
[~, ~, ~, ~, Tall] = fk_4R(theta, L);
P = squeeze(Tall(1:2, 4, :));            % 2x5: origini delle terne 0..4
xa = zeros(2,5); ya = zeros(2,5);
for i = 1:5
    xa(:,i) = P(:,i) + frameScale*Tall(1:2,1,i);
    ya(:,i) = P(:,i) + frameScale*Tall(1:2,2,i);
end
if isempty(h)
    hold(ax, 'on');
    h.links  = plot(ax, P(1,:), P(2,:), '-', 'Color', [0.15 0.25 0.55], 'LineWidth', 5);
    h.joints = plot(ax, P(1,1:4), P(2,1:4), 'o', 'MarkerSize', 9, ...
        'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', [0.1 0.1 0.1], 'LineWidth', 1.5);
    h.ee     = plot(ax, P(1,5), P(2,5), 's', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.85 0.33 0.1], 'MarkerEdgeColor', 'k');
    for i = 1:5
        h.xax(i) = plot(ax, [P(1,i) xa(1,i)], [P(2,i) xa(2,i)], 'r-', 'LineWidth', 1.5);
        h.yax(i) = plot(ax, [P(1,i) ya(1,i)], [P(2,i) ya(2,i)], 'g-', 'LineWidth', 1.5);
        h.lbl(i) = text(ax, P(1,i) + 0.3*frameScale, P(2,i) - 0.4*frameScale, ...
            sprintf('%d', i-1), 'FontSize', 8, 'Color', [0.3 0.3 0.3]);
    end
    plot(ax, 0, 0, 'k^', 'MarkerSize', 12, 'MarkerFaceColor', [0.6 0.6 0.6]);  % base
else
    set(h.links,  'XData', P(1,:),   'YData', P(2,:));
    set(h.joints, 'XData', P(1,1:4), 'YData', P(2,1:4));
    set(h.ee,     'XData', P(1,5),   'YData', P(2,5));
    for i = 1:5
        set(h.xax(i), 'XData', [P(1,i) xa(1,i)], 'YData', [P(2,i) xa(2,i)]);
        set(h.yax(i), 'XData', [P(1,i) ya(1,i)], 'YData', [P(2,i) ya(2,i)]);
        set(h.lbl(i), 'Position', [P(1,i) + 0.3*frameScale, P(2,i) - 0.4*frameScale, 0]);
    end
end
end
