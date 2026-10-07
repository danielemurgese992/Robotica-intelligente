function plot_environment(env, ax)
%PLOT_ENVIRONMENT  Disegna bordi, ostacoli e regione proibita (ostacoli gonfiati).
if nargin < 2, ax = gca; end
hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on');
b = env.bounds;
plot(ax, b([1 2 2 1 1]), b([3 3 4 4 3]), 'k-', 'LineWidth', 1.5);
th = linspace(0, 2*pi, 80);
for j = 1:numel(env.obstacles)
    o = env.obstacles{j};
    if strcmp(o.type, 'circle')
        fill(ax, o.center(1) + o.radius * cos(th), o.center(2) + o.radius * sin(th), ...
             [0.45 0.45 0.5], 'EdgeColor', 'k');
    else
        fill(ax, o.vertices(:, 1), o.vertices(:, 2), [0.45 0.45 0.5], 'EdgeColor', 'k');
    end
end
% contorno dello spazio proibito per il centro del robot (C-ostacoli)
[X, Y] = meshgrid(linspace(b(1), b(2), 200), linspace(b(3), b(4), 200));
C = reshape(clearance([X(:) Y(:)], env), size(X));
contour(ax, X, Y, C, [env.inflation env.inflation], 'LineColor', [0.85 0.4 0.4], 'LineStyle', '--');
xlim(ax, b(1:2)); ylim(ax, b(3:4));
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
end
