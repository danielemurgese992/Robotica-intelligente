function [wR, wL, scale] = saturate_wheels(wR, wL, wmax)
%SATURATE_WHEELS  Saturazione delle ruote che preserva la curvatura.
%   Se una ruota supera wmax, entrambe sono scalate dello stesso fattore:
%   il rapporto wR/wL (e quindi la curvatura w/v del percorso) resta invariato.
m = max(abs(wR), abs(wL));
scale = 1;
if m > wmax
    scale = wmax / m;
    wR = wR * scale;
    wL = wL * scale;
end
end
