function o = make_obstacle(type, a, b)
%MAKE_OBSTACLE  Crea un ostacolo geometrico.
%   o = make_obstacle('circle',  [xc yc], R)
%   o = make_obstacle('polygon', V)   con V matrice Nx2 dei vertici (ordine qualsiasi
%                                    lungo il perimetro, convesso o concavo)
switch lower(type)
    case 'circle'
        assert(numel(a) == 2 && isscalar(b) && b > 0, 'Cerchio: centro [x y] e raggio > 0');
        o = struct('type', 'circle', 'center', a(:)', 'radius', b, 'vertices', []);
    case 'polygon'
        assert(size(a, 2) == 2 && size(a, 1) >= 3, 'Poligono: almeno 3 vertici Nx2');
        o = struct('type', 'polygon', 'center', mean(a, 1), 'radius', [], 'vertices', a);
    otherwise
        error('Tipo ostacolo "%s" non valido (circle | polygon).', type);
end
end
