function animate_4R(res, p)
% ANIMATE_4R  Animazione del robot (drawnow) con terne DH e traiettoria target.
%
%   Ad ogni frame vengono disegnati: robot, terne DH dei giunti, percorso
%   desiderato (tratteggiato), punto target corrente (cerchio verde) e
%   scia dell'end-effector (rosso). Se p.viz.saveVideo = true l'animazione
%   viene salvata in p.viz.videoFile (solo MATLAB, tramite VideoWriter).
L = p.robot.L;
R = sum(L) * 1.05;
fig = figure('Name', 'Animazione robot 4R', 'Color', 'w');
ax = axes('Parent', fig);
axis(ax, 'equal'); axis(ax, [-R R -R R]); grid(ax, 'on'); hold(ax, 'on');
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
plot(ax, res.ref.X(1,:), res.ref.X(2,:), 'k--', 'LineWidth', 1);       % target
hTrace  = plot(ax, NaN, NaN, 'r-', 'LineWidth', 1.2);                   % scia EE
hTarget = plot(ax, NaN, NaN, 'o', 'MarkerSize', 10, 'LineWidth', 2, 'Color', [0 0.6 0]);
h = plot_robot_4R(ax, res.q(:,1), L, [], p.viz.frameScale);
hTitle = title(ax, '');

vw = [];
if p.viz.saveVideo && exist('OCTAVE_VERSION', 'builtin') == 0
    vw = VideoWriter(p.viz.videoFile, 'MPEG-4');
    vw.FrameRate = round(1/(p.sim.dtOut*p.viz.animStep));
    open(vw);
end
idx = [1:p.viz.animStep:numel(res.t), numel(res.t)];
for k = idx
    if ~ishandle(fig), break; end
    h = plot_robot_4R(ax, res.q(:,k), L, h, p.viz.frameScale);
    set(hTrace,  'XData', res.Xee(1,1:k), 'YData', res.Xee(2,1:k));
    set(hTarget, 'XData', res.Xd(1,k),    'YData', res.Xd(2,k));
    set(hTitle, 'String', sprintf('t = %.2f s   |   errore EE = %.1f mm', ...
        res.t(k), 1e3*res.cartErr(k)));
    drawnow;
    if ~isempty(vw), writeVideo(vw, getframe(fig)); end
    if p.viz.pause > 0, pause(p.viz.pause); end
end
if ~isempty(vw), close(vw); fprintf('Video salvato in %s\n', p.viz.videoFile); end
end
