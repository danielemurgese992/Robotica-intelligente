function [env, q0, qf] = build_environment(p)
%BUILD_ENVIRONMENT  Costruisce l'ambiente di lavoro a partire dai parametri.
%   env.bounds     [xmin xmax ymin ymax]
%   env.obstacles  cell array di ostacoli (make_obstacle)
%   env.inflation  raggio robot + margine: distanza minima ammessa
%                  dal centro del robot a ostacoli e bordi
if strcmpi(p.env.scenario, 'custom')
    if isempty(p.env.bounds), error('Scenario custom: specificare p.env.bounds.'); end
    if isempty(p.q0) || isempty(p.qf), error('Scenario custom: specificare p.q0 e p.qf.'); end
    s.bounds = p.env.bounds; s.obstacles = p.env.custom_obstacles;
    s.q0 = p.q0; s.qf = p.qf;
else
    s = scenario_library(p.env.scenario);
end
env.bounds = s.bounds;
if ~isempty(p.env.bounds), env.bounds = p.env.bounds; end
env.obstacles = s.obstacles;
env.inflation = p.robot.radius + p.env.safety_margin;
env.robot_radius = p.robot.radius;

q0 = s.q0; qf = s.qf;
if ~isempty(p.q0), q0 = p.q0; end
if ~isempty(p.qf), qf = p.qf; end
q0 = q0(:)'; qf = qf(:)';
end
