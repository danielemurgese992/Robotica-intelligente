function tf = is_segment_free(a, b, env, step)
%IS_SEGMENT_FREE  Verifica che il segmento a-b sia libero per il robot.
%   Il segmento e' campionato con passo <= step e ogni punto e' verificato.
a = a(1:2); b = b(1:2);
n = max(2, ceil(norm(b - a) / step) + 1);
t = linspace(0, 1, n)';
P = [a(1) + t * (b(1) - a(1)), a(2) + t * (b(2) - a(2))];
tf = all(is_free(P, env));
end
