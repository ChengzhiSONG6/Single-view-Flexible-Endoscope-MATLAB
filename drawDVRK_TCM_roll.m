% Prepared by Li Zheng, May, 2018

function drawDVRK_TCM_roll(thetalist)
% thetalist(1) = pitch
% theta (2) = yaw
% theta (3) = roll
% theta (4) = translate
% theta (5) = bending direction
% thetalist (6) = bending angle

% In the draw function, we first draw the PSM frame and then the active
% joints of the system
[RobotPara, Slist_m, Mlist_m, Slist_f, Mlist_f] = structDVRK_TCM_roll();
L = RobotPara.L;
Lb = RobotPara.Lb;
Ls = RobotPara.Ls;
Lh = RobotPara.Lh;
Lw = RobotPara.Lw;
Lw2 = L-Lw;
N = RobotPara.N;
len = RobotPara.len;


% thetalist = [0 0 0.1 pi/2 pi]';
theta_j = thetalist(6)/N;

% draw the frame
T_f_01 = FKinSpace(Mlist_f{1},Slist_f(:,1),thetalist(1));
T_f_02 = FKinSpace(Mlist_f{2},Slist_f(:,1:2),thetalist(1:2));
T_f_03 = FKinSpace(Mlist_f{3}, Slist_f(:,1:3), [thetalist(1); thetalist(2); -thetalist(2)]);
T_f_04 = FKinSpace(Mlist_f{4}, Slist_f(:,1:4), [thetalist(1); thetalist(2); -thetalist(2); thetalist(2)]);
pJoint_st = [0 0 0 1]';
pJoint_via1 = T_f_01*[0 Lw2 0 1]';
pJoint_via2 = T_f_02*[0 0 Lh 1]';
pJoint_via3 = T_f_03*[0 Lw 0 1]';
pJoint_rcm = T_f_04*[0 0 -Lh 1]';

drawList_f = [pJoint_st pJoint_via1 pJoint_via2 pJoint_via3 pJoint_rcm];
plot3(drawList_f(1,:), drawList_f(2,:), drawList_f(3,:),'-bo','linewidth', 2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','w',...
    'MarkerSize',10);
hold on;

% draw the base frame
length = 0.15;

xs = [[1; 0; 0]*length; 1];
ys = [[0; 1; 0]*length; 1];
zs = [[0; 0; 1]*length; 1];

arrow3(pJoint_st(1:3)', xs(1:3)', ['r'], 0.4, 0.8);
arrow3(pJoint_st(1:3)', ys(1:3)', ['g'], 0.4, 0.8);
arrow3(pJoint_st(1:3)', zs(1:3)', ['b'], 0.4, 0.8);

text(xs(1), xs(2), xs(3), 'X_s');
text(ys(1), ys(2), ys(3), 'Y_s');
text(zs(1), zs(2), zs(3), 'Z_s');

% draw the manipulator
T_m_03 = FKinSpace(Mlist_m{3},Slist_m(:,1:3),thetalist(1:3));
T_m_04 = FKinSpace(Mlist_m{4}, Slist_m(:,1:4),thetalist(1:4));
T_m_05 = FKinSpace(Mlist_m{5},Slist_m(:,1:5),[thetalist(1:5)]);
T_m_06 = FKinSpace(Mlist_m{6}, Slist_m(:,1:6),[thetalist(1:5); theta_j]);
T_m_07 = FKinSpace(Mlist_m{7},Slist_m(:,1:7),[thetalist(1:5); theta_j*ones(2,1)]);
T_m_08 = FKinSpace(Mlist_m{8},Slist_m(:,1:8),[thetalist(1:5);  theta_j*ones(3,1)]);
T_m_09 = FKinSpace(Mlist_m{9},Slist_m(:,1:9),[thetalist(1:5);  theta_j*ones(4,1)]);
T_m_010 = FKinSpace(Mlist_m{10},Slist_m(:,1:10),[thetalist(1:5); theta_j*ones(5,1)]);
T_m_011 = FKinSpace(Mlist_m{11},Slist_m(:,1:11),[thetalist(1:5);  theta_j*ones(6,1)]);
T_m_012 = FKinSpace(Mlist_m{12},Slist_m(:,1:12),[thetalist(1:5);  theta_j*ones(7,1)]);
T_m_013 = FKinSpace(Mlist_m{13},Slist_m(:,1:13),[thetalist(1:5); theta_j*ones(8,1)]);
T_m_014 = FKinSpace(Mlist_m{14},Slist_m(:,1:14),[thetalist(1:5);  theta_j*ones(9,1)]);
T_m_015 = FKinSpace(Mlist_m{15},Slist_m(:,1:15),[thetalist(1:5);  theta_j*ones(10,1)]);

pJoint_tr = T_m_03*[0 0 0 1]';
pJoint_roll = T_m_04*[0 0 0 1]';
pJoint_bst = T_m_05*[0 0 0 1]';
pJoint_flex(:,1) = T_m_06*[0 0 0 1]';
pJoint_flex(:,2) = T_m_07*[0 0 0 1]';
pJoint_flex(:,3) = T_m_08*[0 0 0 1]';
pJoint_flex(:,4) = T_m_09*[0 0 0 1]';
pJoint_flex(:,5) = T_m_010*[0 0 0 1]';
pJoint_flex(:,6) = T_m_011*[0 0 0 1]';
pJoint_flex(:,7) = T_m_012*[0 0 0 1]';
pJoint_flex(:,8) = T_m_013*[0 0 0 1]';
pJoint_flex(:,9) = T_m_014*[0 0 0 1]';
pJoint_flex(:,10) = T_m_015*[0 0 0 1]';

drawList_m = [pJoint_tr pJoint_roll pJoint_bst];
plot3(drawList_m(1,:), drawList_m(2,:), drawList_m(3,:),'-ro','linewidth', 2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','w',...
    'MarkerSize',10);

drawList_flex = [ pJoint_bst pJoint_flex(:,1) pJoint_flex(:,2) pJoint_flex(:,3) pJoint_flex(:,4) pJoint_flex(:,5)...
    pJoint_flex(:,6) pJoint_flex(:,7) pJoint_flex(:,8) pJoint_flex(:,9) pJoint_flex(:,10)];
plot3(drawList_flex(1,:), drawList_flex(2,:), drawList_flex(3,:),'-g','linewidth', 2,...
    'MarkerEdgeColor','k',...
    'MarkerFaceColor','w',...
    'MarkerSize',10);

T_m_016 = FKinSpace_DVRK_TCM(RobotPara, Mlist_m{end}, Slist_m(:,1:4), thetalist);
pJoint_flex_end = T_m_016(1:3,4);
xb = T_m_016 *[[1; 0; 0]*length; 1];
yb = T_m_016 *[[0; 1; 0]*length; 1];
zb = T_m_016 * [[0; 0; 1]*length; 1];
arrow3(pJoint_flex_end', xb(1:3)', ['r'], 0.4, 0.8);
arrow3(pJoint_flex_end', yb(1:3)', ['g'], 0.4, 0.8);
arrow3(pJoint_flex_end', zb(1:3)', ['b'], 0.4, 0.8);

text(xb(1), xb(2), xb(3), 'X_b');
text(yb(1), yb(2), yb(3), 'Y_b');
text(zb(1), zb(2), zb(3), 'Z_b');

axis equal
xlim([-0.3,0.3]);
ylim([0,1.2]);
zlim([-0.5,0.7]);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

