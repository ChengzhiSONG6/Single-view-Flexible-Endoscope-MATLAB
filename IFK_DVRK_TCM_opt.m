% the inverse kinematics model of the DVRK + TCM
% using the second kinematics model

function [thetalist, success, cycle] = IFK_DVRK_TCM_roll(RobotPara, Mlist_m, Slist_m, T, thetalist0, eomg, ev)

thetalist = thetalist0;
i=0;
maxiterations = 50;

Tsb = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist);
Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));

%%%%%%%%%%%%%%%
err_omg = norm(Vs(1:3)) > eomg;
%     err_v = norm(Vs(4:6)) > ev;
%     err =  err_omg || err_v;
err = err_omg;

% plotT(Tsb);
traj(:,i+1) = Tsb(1:3,4);
while err && i<maxiterations
    JJ = Jacobian_DVRK_TCM_roll(RobotPara, Slist_m, thetalist);
    JJJ = JJ(1:3,3:6);
    delta = pinv(JJJ)*Vs(1:3);
    delta = [0;0;delta];
    
    %     %%%%%%%%%%%%%%%%%%
    %     delta = pinv(JJ)*Vs
    
    thetalist = thetalist + delta;
    
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
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    %     if thetalist(5)>pi || thetalist(5) <-pi
    %         thetalist(5) = rem(thetalist(5), pi);
    %     end
    %     if thetalist(6)>=2*pi || thetalist(6)<0
    %         thetalist(6) = rem(thetalist(6),2*pi);
    %     end
    i = i+1;
    Tsb = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist);
    Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    
    err_omg = norm(Vs(1:3)) > eomg;
    %     err_v = norm(Vs(4:6)) > ev;
    %     err =  err_omg || err_v;
    err = err_omg;
    %     drawDVRK_TCM_roll(thetalist);
    traj(:,i+1) = Tsb(1:3,4);
    
    %     pause(0.001);
end

if ~err
    success = 1;
    cycle = i;
else
    success = 0;
    cycle = i;
end

% success = ~ err;
% plot3(traj(1,:), traj(2,:), traj(3,:));
end



