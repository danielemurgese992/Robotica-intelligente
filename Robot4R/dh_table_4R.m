function DH = dh_table_4R(theta, L)
% DH_TABLE_4R  Tabella di Denavit-Hartenberg (convenzione standard) del 4R planare.
%
%   DH = dh_table_4R(theta, L) restituisce una matrice 4x4 le cui righe
%   sono i parametri [a_i  alpha_i  d_i  theta_i] del giunto i:
%
%        i |  a_i  | alpha_i | d_i | theta_i
%       ---+-------+---------+-----+--------
%        1 |  L1   |    0    |  0  | theta1 (variabile)
%        2 |  L2   |    0    |  0  | theta2 (variabile)
%        3 |  L3   |    0    |  0  | theta3 (variabile)
%        4 |  L4   |    0    |  0  | theta4 (variabile)
%
%   Tutti gli assi z_i sono paralleli (alpha_i = 0) e giacciono alla stessa
%   quota (d_i = 0): il moto avviene interamente nel piano XY.
theta = theta(:);  L = L(:);
DH = [L, zeros(4,1), zeros(4,1), theta];
end
