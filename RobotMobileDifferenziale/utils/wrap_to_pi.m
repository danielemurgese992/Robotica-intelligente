function a = wrap_to_pi(a)
%WRAP_TO_PI  Riporta un angolo nell'intervallo (-pi, pi].
a = mod(a + pi, 2*pi) - pi;
a(a == -pi) = pi;
end
