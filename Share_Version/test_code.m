% Prepared by Li Zheng, May, 2018
clear all;
close all;
clc;
% addpath 'E:\Reading\Robotics books\Modern robotics\ModernRobotics-master\code\MATLAB'
addpath 'S:\18_Ph.D-year2\Code\Screw_Theory\ModernRobotics\code\MATLAB'

% structure parameters
[RobotPara, Slist_m, Mlist_m, Slist_f, Mlist_f] = structDVRK_TCM_roll();
% a random desired configuration
% thetalist_dsr = rand(6,1); 
thetalist_dsr = [-pi/4;pi/4;0;0.08;-pi/4;-pi/3];
T_dsr = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist_dsr);
drawDVRK_TCM_roll(thetalist_dsr);
hold on;
grid on
% a random initial configuration
% thetalist0 = rand(6,1);
thetalist0 = [0;0;0;0;0;0];
drawDVRK_TCM_roll(thetalist0);
eomg = 1e-3;
ev = 1e-3;
% success indicates the number of iteration, if fails, success = 0
% the iteration trajectory is plot on the figure.
[thetalist_solve, success] = IFK_DVRK_TCM_roll(RobotPara, Mlist_m, Slist_m, T_dsr, thetalist0, eomg, ev)


