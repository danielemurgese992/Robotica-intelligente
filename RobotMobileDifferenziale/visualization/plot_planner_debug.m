function fig = plot_planner_debug(env, q0, qf, path, info, p)
%PLOT_PLANNER_DEBUG  Mostra la struttura interna del pianificatore.
%   cell: celle occupate / libere e nodi ritratti del cammino
%   prm : roadmap (nodi e archi)
%   rrt : albero di esplorazione
%   apf : curve di livello del potenziale, campo -grad U e minimi locali
fig = figure('Name', ['Pianificatore: ' upper(info.planner)], 'Color', 'w');
ax = gca; plot_environment(env, ax);
switch info.planner
    case 'cell'
        g = info.grid; h = g.res / 2;
        [IY, IX] = find(~g.free);
        cx = g.xc(IX); cy = g.yc(IY); cx = cx(:); cy = cy(:);
        n = numel(cx);
        V = [cx - h, cy - h; cx + h, cy - h; cx + h, cy + h; cx - h, cy + h];
        Fc = [(1:n)', (n + 1:2*n)', (2*n + 1:3*n)', (3*n + 1:4*n)'];
        patch(ax, 'Faces', Fc, 'Vertices', V, 'FaceColor', [1 0.8 0.8], 'EdgeColor', 'none');
        [IY, IX] = find(g.free);
        plot(ax, g.xc(IX), g.yc(IY), '.', 'Color', [0.6 0.75 0.6], 'MarkerSize', 4);
        if isfield(info, 'retracted_nodes')
            plot(ax, info.retracted_nodes(:, 1), info.retracted_nodes(:, 2), 's', ...
                 'Color', [0 0.5 0], 'MarkerSize', 4);
        end
        title(ax, sprintf('Cell decomposition: %d/%d celle libere', info.n_free, info.n_cells));
    case 'prm'
        V = info.nodes; E = info.edges;
        if ~isempty(E)
            xs = [V(E(:, 1), 1) V(E(:, 2), 1) nan(size(E, 1), 1)]';
            ys = [V(E(:, 1), 2) V(E(:, 2), 2) nan(size(E, 1), 1)]';
            plot(ax, xs(:), ys(:), '-', 'Color', [0.75 0.75 0.9]);
        end
        plot(ax, V(:, 1), V(:, 2), '.', 'Color', [0.2 0.2 0.8], 'MarkerSize', 6);
        title(ax, sprintf('PRM: %d nodi, %d archi, %d tentativi', info.n_nodes, info.n_edges, info.attempts));
    case 'rrt'
        V = info.nodes; pr = info.parent; k = find(pr > 0);
        xs = [V(k, 1) V(pr(k), 1) nan(numel(k), 1)]';
        ys = [V(k, 2) V(pr(k), 2) nan(numel(k), 1)]';
        plot(ax, xs(:), ys(:), '-', 'Color', [0.75 0.75 0.9]);
        title(ax, sprintf('RRT: %d nodi in %d iterazioni', info.n_nodes, info.iterations));
    case 'apf'
        b = env.bounds;
        [X, Y] = meshgrid(linspace(b(1), b(2), 120), linspace(b(3), b(4), 120));
        [U, gx, gy] = apf_potential([X(:) Y(:)], qf(1:2), env, p.apf);
        U = reshape(U, size(X));
        contour(ax, X, Y, log(1 + U), 30);
        [Xq, Yq] = meshgrid(linspace(b(1), b(2), 25), linspace(b(3), b(4), 25));
        [~, qx, qy] = apf_potential([Xq(:) Yq(:)], qf(1:2), env, p.apf);
        nq = sqrt(qx.^2 + qy.^2) + eps;
        quiver(ax, Xq(:), Yq(:), -qx ./ nq, -qy ./ nq, 0.4, 'Color', [0.5 0.5 0.5]);
        if isfield(info, 'raw_path')
            plot(ax, info.raw_path(:, 1), info.raw_path(:, 2), '-', 'Color', [0.9 0.6 0], 'LineWidth', 1);
        end
        if ~isempty(info.local_minima)
            plot(ax, info.local_minima(:, 1), info.local_minima(:, 2), 'mx', ...
                 'MarkerSize', 12, 'LineWidth', 2.5);
        end
        title(ax, sprintf('APF: %d minimi locali rilevati, %d fughe', ...
              size(info.local_minima, 1), info.escapes));
end
if ~isempty(path)
    plot(ax, path(:, 1), path(:, 2), 'b-', 'LineWidth', 2);
end
plot_pose(ax, q0, [0 0.7 0], 'q_0');
plot_pose(ax, qf, [0.85 0 0], 'q_f');
end
