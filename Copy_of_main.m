clc;
close all;
clear global;
clear all;
addpath '/home/church/visual trcking/modernrobotics/code/MATLAB'
addpath 'Share_Version'

[RobotPara, Slist_m, Mlist_m, Slist_f, Mlist_f] = structDVRK_TCM_roll();

rosshutdown
rosinit;
P = psm('PSM1');
T_dsr  = eye(4);

q(1) = 0;
q(2) = 0;
q(3) = 0.15;
q(4) = 0;
q(5) = 0;
q(6) = pi/6;
goal_reached = false;
while goal_reached == false
    goal_reached = P.move_joint(q);
    pause(0.5);
end
q_home = P.get_state_joint_current();
% 
global obj
obj = webcam('USB2.0 PC CAMERA: USB2.0 PC CAM');
% obj = videoinput('winvideo', 1,'YUY2_640x480');
% preview(obj);
% triggerconfig(obj, 'manual');
% start(obj);


for i=1:1
    u=100;
    v=300;
%     [u,v]  = KK;
    error1 = sqrt((u-320)^2);
    error2 = sqrt((v-240)^2);
    eomg = 1e-3;
    ev = 1e-3;
    
    TTTT=0;
    O=1;
    while(error1>60 || error2>60)
        tic
        u=10;
        v=235;
%         [u,v]  = KK;
        error1 = sqrt((u-320)^2);
        error2 = sqrt((v-240)^2);
        
        uu=[u,v];
        [theta, phi]        = T(uu)
        
        %%%%%% THETALIST(3) IS io INSERTION. 
        thetalist_current   = [0,0,0,0.08,0,pi/6]';
%         thetalist_current   = P.get_state_joint_current();
%         l4ml2               = thetalist_current(5);
%         l3ml1               = thetalist_current(6);
%         [phi_cur,theta_cur ]= Cable2Joint(l4ml2,l3ml1);
%         thetalist_current(5)= phi_cur;
%         thetalist_current(6)= theta_cur;
        
        
%         [Slist,M,tlist,~]   = construct(thetalist_current);
%         T_current           = FKinSpace_new(M,Slist,tlist);
        io                  = thetalist_current(3);
        roll                = thetalist_current(4);
        thetalist_current(3)= roll;
        thetalist_current(4)= io;
        T_current           = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist_current)
        
        
        p_current           = T_current(1:3,4);
        R_current           = T_current(1:3,1:3);
        x_cur               = p_current(1);
        y_cur               = p_current(2);
        z_cur               = p_current(3);
%         if abs(phi)<pi/2
%         theta = -theta;
%         else
%         end
 phi=-phi
%        if phi>0 & phi<pi/2
%            phi=-phi
%        end
%        if phi<0 & phi>-pi/2
%            phi=-phi
%        end   

        R_dsr               = R_current* rotz((phi)*180/pi)*roty((theta/3)*180/pi)* rotz((-phi)*180/pi);
        
        
        T_dsr(1:3,1:3)      = R_dsr;
        T_dsr(1:3,4)        = p_current
        [thetalist_solve, success] = IFK_DVRK_TCM_roll(RobotPara, Mlist_m, Slist_m, T_dsr, thetalist_current, eomg, ev);
%         t =1;
%         algorithm   = 'Numerical Inverse Kinematics';
%         [q_act, cycle_hi, Jac_hist, dJac_hist, p_act, success]   = IKinSpace_new(T_dsr,thetalist_current,t,algorithm);
% figure (4)        
%  drawDVRK_TCM_roll(thetalist_current);        
% drawDVRK_TCM_roll(thetalist_solve);
        roll                = thetalist_solve(3);
        io                  = thetalist_solve(4);
        
        thetalist_solve(3)  = io;
        thetalist_solve(4)  = roll;

        q_act              = thetalist_solve;
        cycle_hist(:,1)    = success;
        
        phi_act         = q_act(5);
        theta_act       = q_act(6);
        [l4ml2,l3ml1] = Joint2Cable(phi_act,theta_act);
        
%         q_cur           = P.get_state_joint_current();
        q_act(5)        = l4ml2;
        q_act(6)        = l3ml1 ;
        q_act
        goal_reached = false;
        while goal_reached == false
            goal_reached = P.move_joint(q_act);
             pause(0.3);
        end
        O=O+1;
        TTTT=TTTT+toc
    end
    O;
    TTTT;
end



