function L = path_length(P)
%PATH_LENGTH  Lunghezza di una polilinea P (Nx2).
if size(P, 1) < 2, L = 0; return; end
L = sum(sqrt(sum(diff(P, 1, 1).^2, 2)));
end
