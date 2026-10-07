function A = dh_matrix(a, alpha, d, theta)
% DH_MATRIX  Matrice di trasformazione omogenea A_i (convenzione DH standard).
%
%   A = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
%
%       [ cos(th)  -sin(th)*cos(al)   sin(th)*sin(al)   a*cos(th) ]
%   A = [ sin(th)   cos(th)*cos(al)  -cos(th)*sin(al)   a*sin(th) ]
%       [   0          sin(al)            cos(al)            d     ]
%       [   0            0                  0                1     ]
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
% (righe concatenate con vertcat: compatibile anche con variabili simboliche
%  in GNU Octave)
A = vertcat([ ct, -st*ca,  st*sa, a*ct ], ...
            [ st,  ct*ca, -ct*sa, a*st ], ...
            [  0,     sa,     ca,    d ], ...
            [  0,      0,      0,    1 ]);
end
