



clear all;
close all;
clc;
% addpath 'E:\Reading\Robotics books\Modern robotics\ModernRobotics-master\code\MATLAB'
addpath 'S:\18_Ph.D-year2\Code\Screw_Theory\ModernRobotics\code\MATLAB'

% structure parameters
% theta (5) = bending direction phi------------------->S5
% theta (6) = bending angle theta  ------------------->S6
[RobotPara, Slist_m, Mlist_m, Slist_f, Mlist_f] = structDVRK_TCM_roll();
% a random desired configuration
% thetalist_dsr = rand(6,1); 
thetalist_dsr = [0.0; 0.0; pi/4; 0.08; 0.0; 0.01];
T_dsr = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m, thetalist_dsr);
figure(1);
drawDVRK_TCM_roll(thetalist_dsr);
view(40,150);
hold on;
grid on
% a random initial configuration
% thetalist0 = rand(6,1);
thetalist_ini = [0; 0; 0; 0.08; 0.0; 0.01];
figure(2);
drawDVRK_TCM_roll(thetalist_ini);
eomg = 1e-3;
ev = 1e-3;
% success indicates the number of iteration, if fails, success = 0
% the iteration trajectory is plot on the figure.
[thetalist_solve, success] = IFK_DVRK_TCM_roll(RobotPara, Mlist_m, Slist_m,...
    T_dsr, thetalist_ini, eomg, ev)

view(40,150);
hold on;
grid on
