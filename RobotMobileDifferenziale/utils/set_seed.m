function set_seed(seed)
%SET_SEED  Imposta il generatore casuale (compatibile MATLAB e GNU Octave).
if exist('OCTAVE_VERSION', 'builtin')
    rand('seed', seed); randn('seed', seed); %#ok<RAND>
    rand('state', seed); randn('state', seed); %#ok<RAND>
else
    rng(seed);
end
end
