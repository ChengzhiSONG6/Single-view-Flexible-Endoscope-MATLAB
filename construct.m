function [Slist,M,tlist,Jf] = construct(thetalist)

%%%%%%% reference to Modern Robotics P147

global Lf Ltool LRCC
Lf = 100/1e3; % m
Ltool = 416.2/1e3;
LRCC  = 431.8/1e3;
M =  [0, -1, 0, 0;
    -1,  0, 0, 0;
    0,  0, -1,-(Lf+Ltool-LRCC);
    0,  0, 0, 1];

% Outer Yaw
w1 = [0, -1, 0]';
q1 = [0, 0, 0]';
v1 = -(VecToso3(w1))*q1;
s1 = [w1;v1];
% Outer Pitch
w2 = [-1, 0, 0]';
q2 = [0, 0, 0]';
v2 = -(VecToso3(w2))*q2;
s2 = [w2;v2];
% I/O insertion
w3 = [0, 0, 0]';
v3 = [0, 0, -1]';
% v3 = -(VecToso3(w3))*q3;
s3 = [w3;v3];
% Outer Roll
w4 = [0, 0, -1]';
q4 = [0, 0, 0]';
v4 = -(VecToso3(w4))*q4;
s4 = [w4;v4];
% Flexible CTM get screw axis by logarithm
% % syms phi theta_bending
% % T4b = [cos(phi)^2*cos(theta_bending)+sin(phi)^2, sin(phi)*cos(phi)*(cos(theta_bending)-1), cos(phi)*sin(theta_bending), cos(phi)*(1-cos(theta_bending))*Lf/theta_bending;
% %     sin(phi)*cos(phi)*(cos(theta_bending)-1), cos(phi)^2+cos(theta_bending)*sin(phi)^2, sin(phi)*sin(theta_bending), sin(phi)*(1-cos(theta_bending))*Lf/theta_bending;
% %     -cos(phi)*sin(theta_bending),                -sin(phi)*sin(theta_bending),                cos(theta_bending),             sin(theta_bending)*Lf/theta_bending;
% %     0,                                  0,                               0,                         1];
% % T4b = double(subs(T4b,[phi,theta_bending],[thetalist(5),thetalist(6)]));
% %
% % Ts4 = [0, -1, 0, 0;
% %     -1, 0, 0, 0;
% %     0, 0, -1, LRCC-Ltool;
% %     0, 0, 0, 1];
% %
% % Exp5 = Ts4*T4b*pinv(M);
% % R   = Exp5(1:3,1:3);
% % p   = Exp5(1:3,4);
% % theta_logso3 = acos(1/2*(trace(R)-1));
% % theta = theta_logso3;
% % w_hat = 1/2/sin(theta)*(R-R');
% % w_f   = so3ToVec(w_hat);
% % G_inv = eye(3)/theta - 1/2*w_hat + (1/theta - 1/2*cot(theta/2))*w_hat^2;
% % v_f   = G_inv*p;
% %

% Slist = -cos(phi)
%         sin(phi)
%         0
%         sin(phi)*(Lf/2 - LRCC + Ltool)
%         cos(phi)*(Lf/2 - LRCC + Ltool)
%         -(Lf*(2*sin(theta/2) - theta*cos(theta/2)))/(2*theta*sin(theta/2))


phi   = thetalist(5);
theta = thetalist(6);
w_f   = [-cos(phi); sin(phi);0];
v_f   = [sin(phi)*(Lf/2 - LRCC + Ltool);...
    cos(phi)*(Lf/2 - LRCC + Ltool);...
    -(Lf*(2*sin(theta/2) - theta*cos(theta/2)))/(2*theta*sin(theta/2))];

s_f   = [w_f;v_f];

% J_fMMT =
%                          J_phi                            J_theta
% [                  -cos(phi)*sin(theta),                              -sin(phi)]
% [                  -sin(phi)*sin(theta),                               cos(phi)]
% [                        1 - cos(theta),                                      0]
% [ -(Lf*sin(phi)*(cos(theta) - 1))/theta, (Lf*cos(phi)*(cos(theta) - 1))/theta^2]
% [  (Lf*cos(phi)*(cos(theta) - 1))/theta, (Lf*sin(phi)*(cos(theta) - 1))/theta^2]
% [                                     0,      (Lf*(theta - sin(theta)))/theta^2]

% J1 = w_f*Lf/theta*(cos(theta)-1);
% J2 = Lf/theta^2* (w_fdiff*(1-cos(theta)) + VecToso3(w_f)*w_fdiff*(theta-sin(theta)));
% J3 = w_fdiff*sin(theta) +  VecToso3(w_f)*w_fdiff*(1-cos(theta));
% J4 = w_f;

Jw_phi      = [-cos(phi)*sin(theta); -sin(phi)*sin(theta); 1-cos(theta)];
Jw_theta    = [-sin(phi); cos(phi); 0 ];
Jv_phi      = Lf*(cos(theta)-1)/theta * [-sin(phi); cos(phi); 0 ];
Jv_theta    = Lf/theta^2 * [cos(phi)*(cos(theta)-1); sin(phi)*(cos(theta)-1); theta-sin(theta)];
Jf = [Jw_phi,Jw_theta;Jv_phi,Jv_theta];

Slist = [s1,s2,s3,s4,s_f];
tlist(:,1) = [thetalist(1:4);theta];


