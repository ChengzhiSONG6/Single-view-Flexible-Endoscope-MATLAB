
function Js = Jacobian_DVRK_TCM_roll(RobotPara, Slist_m, thetalist)
N = RobotPara.N;
Lb = RobotPara.Lb;
L = RobotPara.L;
theta = thetalist(end);
phi = thetalist(end-1);

% % use the Adjoint transformation to obtain S in {s}
% if abs(theta)<1e-3
%     r = 1e12;
% else    
% r = Lb/theta-Lb/2/tan(theta/2);
% end
% q = [r*cos(phi); r*sin(phi); Lb/2];
% omgb = [-sin(phi); cos(phi); 0];
% vb = -cross(omgb, q);
% Sb=[omgb; vb];
% Tsb=[-1 0 0 0; 0 1 0 L; 0 0 -1 0; 0 0 0 1];
% Sf = Adjoint(Tsb)*Sb;

% in the local frame
omg = [-sin(phi); cos(phi); 0];
domg = [-cos(phi); -sin(phi); 0];
vomg_domg = [0; 0; 1];

if abs(theta)<1e-3
    Jw1 = domg*sin(theta)+vomg_domg*(1-cos(theta));
    Jw2 = omg;
    Jv1 = omg*Lb*0;
    Jv2 = Lb*domg/2+vomg_domg*0;
else
    Jw1 = domg*sin(theta)+vomg_domg*(1-cos(theta));
    Jw2 = omg;
    Jv1 = omg*Lb*(cos(theta)-1)/theta;
    Jv2 = Lb/theta/theta*(domg*(1-cos(theta))+vomg_domg*(theta-sin(theta)));
end
Jf = [Jw1 Jw2; Jv1 Jv2];

Jm = Slist_m(:,1:4);
T = eye(4);
for i = 2:length(thetalist)-2
    T = T * MatrixExp6(VecTose3(Slist_m(:,i - 1) * thetalist(i - 1)));
	Jm(:,i) = Adjoint(T) * Slist_m(:,i);
end
T = T * MatrixExp6(VecTose3(Slist_m(:,4) * thetalist(4)));
T = T * [-1 0 0 0; 0 1 0 L; 0 0 -1 0; 0 0 0 1];
Jf = Adjoint(T)*Jf;
Js = [Jm, Jf];
Js = Js;




% A = L*(N-1)/(2*N);
% Phi = 0;
% omgf = [-sin(Phi); cos(Phi); 0];
% 
% % B = 0.5*(1/N/tan(Theta/2/N)-1/tan(Theta/2));
% if abs(Theta>1e-3)
%     B = 0.5*(1/N/tan(Theta/2/N)-1/tan(Theta/2));
% else
%     B = 10e9;
% end
% 
% Tf = Mlist_m{5};
% Rf = Tf(1:3,1:3);
% omgf = Rf*omgf;
% qf = [B*L; 0; A];
% qf = Tf*[qf;1];
% vf = -cross(omgf,qf(1:3));
% S_f = [omgf; vf];
% Slist = [Slist_m(:,1:4) S_f];
end