% Prepared by Li Zheng, May, 2018
% the inverse kinematics model of the DVRK + TCM
% using the second kinematics model

function [thetalist, success] = IFK_DVRK_TCM_roll(RobotPara, Mlist_m, Slist_m, T, thetalist0, eomg, ev)

thetalist = thetalist0;
i=0;
maxiterations = 20;

Tsb = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist);

Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));

err = norm(Vs(1:3)) > eomg;
% plotT(Tsb);
traj(:,i+1) = Tsb(1:3,4);

while err && i<maxiterations
    
    JJ = Jacobian_DVRK_TCM_roll(RobotPara, Slist_m, thetalist);
    JJ = JJ(1:3,:);
    X1 = [JJ(1,:),JJ(2,:),JJ(3,:),Vs(1:3)'];
    
    x0 = [0,0,0,0,0.1,0];
    % options=optimset('LargeScale','Display','on','iter');
    
    options=optimset('LargeScale','on','Display', 'off');
    [x,~]=lsqnonlin(@B,x0,[],[],options,X1);

    delta     =  x';
    
    thetalist = thetalist + delta;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if thetalist(1)> pi/2 || thetalist(1) <-pi/2
        thetalist(1) = rem(thetalist(1), pi/2);
    end
    if thetalist(2)> pi/2 || thetalist(2) <-pi/2
        thetalist(2) = rem(thetalist(2), pi/2);
    end
    if thetalist(3)> pi || thetalist(3) <-pi
        thetalist(3) = rem(thetalist(3), pi);
    end
    %%%
    if thetalist(4)> 0.24
        thetalist(4) = rem(thetalist(4),0.12);
    end
    if thetalist(5)> pi || thetalist(5) <-pi
        thetalist(5) = rem(thetalist(5), pi);
    end
    if thetalist(6)> pi || thetalist(6) <-pi
        thetalist(6) = rem(thetalist(6), pi);
    end
    phi    = thetalist(5);
    theta  = thetalist(6);
    if theta < 0
        theta = -theta;
        phi = phi+pi;
    else
    end
    if phi >= pi
        phi = phi - 2*pi;
    elseif phi < -pi
        phi = 2*pi + phi;
    end
    thetalist(5) = phi;
    thetalist(6) = theta;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    i   = i+1;
    Tsb = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist);
    Vs  = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
%     thetalist(4)=0;
    err = norm(Vs(1:3)) > eomg;
    %     drawDVRK_TCM_roll(thetalist);
    traj(:,i+1) = Tsb(1:3,4);
    
%     pause(0.5);
end
  
if ~err
    success = i;
else
    success = ~err;
end
%
% success = ~ err;
% plot3(traj(1,:), traj(2,:), traj(3,:));
end



