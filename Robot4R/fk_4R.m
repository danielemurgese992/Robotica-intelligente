function [T, x, y, phi, Tall] = fk_4R(theta, L)
% FK_4R  Cinematica diretta del manipolatore planare 4R.
%
%   [T, x, y, phi] = fk_4R(theta, L)
%   [T, x, y, phi, Tall] = fk_4R(theta, L)
%
%   Ingressi:
%     theta : vettore 4x1 (o 1x4) delle variabili di giunto [rad]
%     L     : vettore 4x1 (o 1x4) delle lunghezze dei link   [m]
%   Uscite:
%     T     : matrice omogenea 4x4 T = A1*A2*A3*A4 (terna 4 rispetto alla base)
%     x, y  : posizione dell'end-effector [m]
%     phi   : orientamento dell'end-effector = theta1+theta2+theta3+theta4 [rad]
%     Tall  : array 4x4x5 con le terne T0^0 (identita'), T0^1, ..., T0^4
%             (utile per disegnare il robot e le terne DH)
DH   = dh_table_4R(theta, L);
Tall = zeros(4,4,5);
Tall(:,:,1) = eye(4);
T = eye(4);
for i = 1:4
    A = dh_matrix(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
    T = T * A;
    Tall(:,:,i+1) = T;
end
x   = T(1,4);
y   = T(2,4);
phi = atan2(T(2,1), T(1,1));   % = theta1+theta2+theta3+theta4 (ricondotto in (-pi, pi])
end
