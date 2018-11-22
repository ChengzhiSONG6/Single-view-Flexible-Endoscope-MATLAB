% Prepared by Li Zheng, May, 2018

function T = FKinSpace_DVRK_TCM(RobotPara, M, Slist, thetalist)
% Mlist shows the initial configurations
% Slist shows the screws, for the bending section the screw is calculated
% independently
% thetalist shows all the joints' rotations, for the bending section, it
% contains two angles: the bending direction and the bending angle
% thetalist = [pitch, yaw, roll, trans, bend, dir]
L = RobotPara.L;
Lb = RobotPara.Lb;


T = M;
phi = thetalist(end-1);
theta = thetalist(end);

% use the Adjoint transformation to obtain S in {s}
if abs(theta)<1e-3
    r = 1e12;
else
    r = Lb/theta-Lb/2/tan(theta/2);
end
q = [r*cos(phi); r*sin(phi); Lb/2];
omgf = [-sin(phi); cos(phi); 0];
vf = -cross(omgf, q);
Sf=[omgf; vf];
Tsf=[-1 0 0 0; 0 1 0 L; 0 0 -1 0; 0 0 0 1];
S = Adjoint(Tsf)*Sf;

T = MatrixExp6(VecTose3(S*theta)) * T;
for i = size(thetalist,1)-2:-1:1
    T = MatrixExp6(VecTose3(Slist(:,i) * thetalist(i))) * T;
end
Tbc = [1 0 0 0; 0 1 0 0; 0 0 1 0.035; 0 0 0 1];

T = T*Tbc;

end