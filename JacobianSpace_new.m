%*** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***

function Js = JacobianSpace_new(Slist, tlist, Jf)
% Takes Slist: The joint screw axes in the space frame when the manipulator
%              is at the home position, in the format of a matrix with the
%              screw axes as the columns,
%       thetalist: A list of joint coordinates.
% Returns the corresponding space Jacobian (6xn real numbers).
% Example Input:
%{
  clear; clc;
  Slist = [[0; 0; 1;   0; 0.2; 0.2], ...
           [1; 0; 0;   2;   0;   3], ...
           [0; 1; 0;   0;   2;   1], ...
           [1; 0; 0; 0.2; 0.3; 0.4]];
  thetalist = [0.2; 1.1; 0.1; 1.2];
  Js = JacobianSpace(Slist, thetalist)
%}
% Output:
% Js =
%         0    0.9801   -0.0901    0.9575
%         0    0.1987    0.4446    0.2849
%    1.0000         0    0.8912   -0.0453
%         0    1.9522   -2.2164   -0.5116
%    0.2000    0.4365   -2.4371    2.7754
%    0.2000    2.9603    3.2357    2.2251
global LRCC Ltool
Js = Slist;
T = eye(4);
Ts4 = [0, -1, 0, 0;
       -1, 0, 0, 0;
       0,  0, -1, LRCC-Ltool;
       0,  0, 0, 1];
for i = 2: size(Slist,2)
    T = T * MatrixExp6(VecTose3(Slist(:, i - 1) * tlist(i - 1)));
    Js(:, i) = Adjoint(T) * Slist(:, i);
end
% Adj(Ts1*T12*T23*T34)
T = T*Ts4;
Jf = Adjoint(T)*Jf;
Js(:,i:i+1) =  Jf;
end

