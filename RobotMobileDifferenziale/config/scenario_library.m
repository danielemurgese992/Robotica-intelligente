function s = scenario_library(name)
%SCENARIO_LIBRARY  Ambienti predefiniti.
%
%   s = scenario_library(name) restituisce una struttura con:
%     s.bounds    [xmin xmax ymin ymax]
%     s.obstacles cell array di ostacoli; ogni ostacolo e' una struct
%                 creata con make_obstacle:
%                   make_obstacle('circle',  [xc yc], R)
%                   make_obstacle('polygon', [x1 y1; x2 y2; ...])
%     s.q0, s.qf  configurazioni iniziale e finale suggerite [x y theta]
%
%   Scenari: 'base'   ostacoli misti circolari e poligonali
%            'narrow' muro con un passaggio stretto
%            'trap'   ostacolo a U che genera un minimo locale per APF

circ = @(c, R) make_obstacle('circle', c, R);
poly = @(V) make_obstacle('polygon', V);
rect = @(x1, y1, x2, y2) make_obstacle('polygon', [x1 y1; x2 y1; x2 y2; x1 y2]);

switch lower(name)
    case 'base'
        s.bounds = [0 10 0 10];
        s.obstacles = { circ([3.0 3.0], 1.0), ...
                        circ([7.2 2.4], 0.8), ...
                        rect(4.6, 4.8, 6.0, 8.0), ...
                        poly([1.2 6.0; 3.4 6.0; 2.3 8.0]), ...
                        rect(7.0, 6.4, 8.8, 7.1), ...
                        circ([8.2 4.6], 0.55) };
        s.q0 = [0.8 0.8 0];
        s.qf = [9.2 9.2 pi/2];
    case 'narrow'
        s.bounds = [0 10 0 10];
        s.obstacles = { rect(4.6, 0.0, 5.3, 4.4), ...
                        rect(4.6, 5.6, 5.3, 10.0), ...
                        circ([2.5 7.0], 0.9), ...
                        circ([7.5 3.0], 0.9), ...
                        poly([7.0 7.0; 8.6 6.6; 8.2 8.4]) };
        s.q0 = [1.0 2.0 0];
        s.qf = [9.0 8.8 0];
    case 'trap'
        s.bounds = [0 10 0 10];
        s.obstacles = { rect(6.0, 3.0, 6.4, 7.0), ...   % fondo della U
                        rect(4.0, 6.6, 6.4, 7.0), ...   % braccio superiore
                        rect(4.0, 3.0, 6.4, 3.4) };     % braccio inferiore
        s.q0 = [1.0 5.0 0];
        s.qf = [9.0 5.0 0];
    otherwise
        error('Scenario "%s" sconosciuto. Usa base | narrow | trap | custom.', name);
end
end
