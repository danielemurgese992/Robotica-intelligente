function pt = point_on_path(P, s_cum, s)
%POINT_ON_PATH  Punto alla ascissa curvilinea s lungo la polilinea P.
s = min(max(s, 0), s_cum(end));
pt = [interp1(s_cum, P(:, 1), s), interp1(s_cum, P(:, 2), s)];
end
