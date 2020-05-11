clc
clear

syms p0 p1
syms n1 n2
syms x1 x2 x3 x4
syms w11 w12 w13 w14 w21 w22 w23 w24 w31 w32 w33 w34 w41 w42 w43 w44

Mi = [ 1, 0, p0, -p1;
       0, 1, p1, p0]
   
% 1.a)  define the matrix and check if it is symmetric
%       --> the transposed and normal matrix have to be equal
Bi = Mi.'*Mi
is_symmetric = isequal(Bi, Bi.')

% 1.b)  check if it is positive semidefinite
%       --> all the Eigenvalues have to be >= 0

eigen_Bi = eig(Bi)
% since both the squares are always >= 0 the matrix is positive
% semi-definit

% 2.a)
x = [x1; x2; x3; x4]

W = [w11,w12,w13,w14; w21,w22,w23,w24; w31,w32,w33,w34; w41,w42,w43,w44]

% calculate the formula for the coefficient comparison
Form = x.'*W*x == 1
Form_2 = x3^2+x4^2 == 1 

% --> from the coefficient comparison follows W
W = [ 0, 0, 0, 0;
      0, 0, 0, 0;
      0, 0, 1, 0;
      0, 0, 0, 1; ]

% calculate and define M symbolically
ni = [n1; n2] 
Ci = ni*ni.'
M = Mi.'*Ci*Mi
  
% 2.b) check W and M if they are positiv semi-definit
eigen_W = eig(W)
%  --> all eigenvalues greater or equal 0 so W is positive semi-definit

eigen_M = eig(M)
%  --> all eigenvalues greater or equal 0 so M is positive semi-definit