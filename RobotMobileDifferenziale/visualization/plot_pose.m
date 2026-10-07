function plot_pose(ax, q, color, label, len)
%PLOT_POSE  Disegna una configurazione [x y theta] come punto + freccia.
if nargin < 5, len = 0.6; end
plot(ax, q(1), q(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', color, 'MarkerEdgeColor', 'k');
quiver(ax, q(1), q(2), len * cos(q(3)), len * sin(q(3)), 0, 'Color', color, ...
       'LineWidth', 2, 'MaxHeadSize', 0.8);
text(ax, q(1) + 0.15, q(2) - 0.3, label, 'FontWeight', 'bold');
end
