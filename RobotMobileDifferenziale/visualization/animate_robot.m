function animate_robot(res, path, q0, qf, env, p)
%ANIMATE_ROBOT  Animazione del moto (corpo, ruote, prua e scia).
%   Con p.viz.save_gif = true salva una GIF in p.viz.output_dir.
fig = figure('Name', 'Animazione', 'Color', 'w'); ax = gca;
plot_environment(env, ax);
plot(ax, path(:, 1), path(:, 2), 'b--', 'LineWidth', 1.2);
plot_pose(ax, q0, [0 0.7 0], 'q_0'); plot_pose(ax, qf, [0.85 0 0], 'q_f');
S = robot_shape(res.q(1, :), p.robot);
hB = fill(ax, S.body(:, 1), S.body(:, 2), [0.3 0.6 1], 'FaceAlpha', 0.6);
hR = fill(ax, S.wheelR(:, 1), S.wheelR(:, 2), 'k');
hL = fill(ax, S.wheelL(:, 1), S.wheelL(:, 2), 'k');
hH = plot(ax, S.head(:, 1), S.head(:, 2), 'r-', 'LineWidth', 2);
hT = plot(ax, res.q(1, 1), res.q(1, 2), 'r-', 'LineWidth', 1.5);
hG = plot(ax, res.target(1, 1), res.target(1, 2), 'g+', 'MarkerSize', 10, 'LineWidth', 2);
gif = '';
if p.viz.save_gif
    if ~exist(p.viz.output_dir, 'dir'), mkdir(p.viz.output_dir); end
    gif = fullfile(p.viz.output_dir, p.viz.gif_file);
end
frames = unique([1:p.viz.frame_skip:numel(res.t), numel(res.t)]);
for k = frames
    if ~ishandle(fig), return; end
    S = robot_shape(res.q(k, :), p.robot);
    set(hB, 'XData', S.body(:, 1), 'YData', S.body(:, 2));
    set(hR, 'XData', S.wheelR(:, 1), 'YData', S.wheelR(:, 2));
    set(hL, 'XData', S.wheelL(:, 1), 'YData', S.wheelL(:, 2));
    set(hH, 'XData', S.head(:, 1), 'YData', S.head(:, 2));
    set(hT, 'XData', res.q(1:k, 1), 'YData', res.q(1:k, 2));
    set(hG, 'XData', res.target(k, 1), 'YData', res.target(k, 2));
    title(ax, sprintf('t = %.2f s   \\omega_R = %.1f   \\omega_L = %.1f rad/s', ...
          res.t(k), res.wR(k), res.wL(k)));
    drawnow;
    if ~isempty(gif)
        try
            fr = getframe(fig); [A, map] = rgb2ind(frame2im(fr), 128);
            if k == frames(1)
                imwrite(A, map, gif, 'gif', 'LoopCount', inf, 'DelayTime', 0.03);
            else
                imwrite(A, map, gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.03);
            end
        catch err
            warning('Salvataggio GIF non riuscito: %s', err.message); gif = '';
        end
    end
    pause(p.viz.pause);
end
end
