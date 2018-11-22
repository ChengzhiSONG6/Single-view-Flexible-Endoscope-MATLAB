% Prepared by Li Zheng, May, 2018
function [RobotPara, Slist_m, Mlist_m, Slist_f, Mlist_f] = structDVRK_TCM_roll()
% structure parameters
RobotPara.L = 0.6; % the length of psm
RobotPara.Lb = 0.035; % bending section length
RobotPara.Ls = 0.4162; % shaft length
RobotPara.Lh = 0.2;
RobotPara.Lw = 0.3;
RobotPara.N = 10;
RobotPara.len = RobotPara.Lb/RobotPara.N;

L = RobotPara.L;
Lb = RobotPara.Lb;
Ls = RobotPara.Ls;
Lh = RobotPara.Lh;
Lw = RobotPara.Lw;
Lw2 = L-Lw;
N = RobotPara.N;
len = RobotPara.len;

% initial poistion of the joints
% joints on the frame
pJoint_st_0 = [0;0;0];
pJoint_via1_0 = [0;Lw2;0]; 
pJoint_via2_0 = [0; Lw2; Lh];
pJoint_via3_0 = [0; L; Lh];
pJoint_rcm_0 = [0; L; 0];
% joints on the manipulator
pJoint_tr_0 = [0;L;Ls];
pJoint_roll_0 = [0; L; Ls];
pJoint_bst_0 = [0; L; 0];
for i=1:N
    pJoint_flex(:,i)=[0;L;-i*len];
end
pJoint_end_0 = [0; L; -Lb];

% screw of the frame
% thetalist(1) = pitch ------------------------> S1
% thetalist (2) = yaw ----------------------------> S2
omg1 = [0;1;0]; q1 = [0;Lw2;0]; v1 = -cross(omg1,q1);
omg2 = [1; 0;0]; q2 = [0; Lw2; 0]; v2 = -cross(omg2, q2);
q3 = [0; Lw2; Lh]; q4 = [0; L; Lh]; % for the via joints
S1 = [omg1; v1];
S2 = [omg2; v2]; 
S3 = [omg2; -cross(omg2,q3)];
S4 = [omg2; -cross(omg2,q4)];
Slist_f = [S1 S2 S3 S4];
Mlist_f = cell(1,5);
R = eye(3);
Mlist_f{1} = RpToTrans(R,pJoint_st_0);
Mlist_f{2} = RpToTrans(R,pJoint_via1_0);
Mlist_f{3} = RpToTrans(R,pJoint_via2_0);
Mlist_f{4} = RpToTrans(R,pJoint_via3_0);
Mlist_f{5} = RpToTrans(R,pJoint_rcm_0);

% screw of the shaft
% thetalist (3) = roll------------------------------> S3
% thetalist (4) = translate ------------------------------> S4
omg3 = [0; 0; -1]; q3 = [0; L; 0]; v3 = -cross(omg3,q3);
omg4 = [0; 0; 0]; v4 = [0; 0; -1];

% screw of the bending section
% theta (5) = bending direction----------------------> S5
% theta (6) = bending angle------------------->S6
omg5 = [0;0; -1]; v5 = [-L;0;0];                                          % bending plane
omg6 = [0; 1; 0]; q6 = [0; L; 0]; v6 = -cross(omg6,q6);  % base of the bending section
omg7 = [0;1;0]; q7 = [0;L;-len]; v7=-cross(omg7,q7);
omg8 = [0;1;0]; q8 = [0;L;-2*len]; v8=-cross(omg8,q8);
omg9 = [0;1;0];q9 = [0;L;-3*len]; v9=-cross(omg9,q9);
omg10 = [0;1;0]; q10 = [0;L;-4*len]; v10=-cross(omg10,q10);
omg11 = [0;1;0]; q11 = [0;L;-5*len]; v11=-cross(omg11,q11);
omg12 = [0;1;0]; q12 = [0;L;-6*len]; v12=-cross(omg12,q12);
omg13 = [0;1;0]; q13 = [0;L;-7*len]; v13=-cross(omg13,q13);
omg14 = [0;1;0]; q14 = [0;L;-8*len]; v14=-cross(omg14,q14);
omg15 = [0;1;0]; q15= [0;L;-9*len]; v15=-cross(omg15,q15);
omg16 = [0;1;0]; q16 = [0;L;-10*len]; v16=-cross(omg16,q16);

% Slist of the manipulator
omg1 = [0;1;0]; q1 = [0;L;0]; v1 = -cross(omg1,q1);
omg2 = [1; 0;0]; q2 = [0; L; 0]; v2 = -cross(omg2, q2);
S1 = [omg1; v1];
S2 = [omg2; v2];
S3 = [omg3; v3];
S4 = [omg4; v4];
S5 = [omg5; v5];
S6 = [omg6; v6];
S7 = [omg7; v7];
S8 = [omg8; v8];
S9 = [omg9; v9];
S10 = [omg10; v10];
S11 = [omg11; v11];
S12 = [omg12; v12];
S13 = [omg13; v13];
S14 = [omg14; v14];
S15 = [omg15; v15];
S16 = [omg16; v16];
Slist_m=[S1 S2 S3 S4 S5 S6 S7 S8 S9 S10 S11 S12 S13 S14 S15 S16];

% initial configuration of the joints
Mlist_m = cell(1,15);
R = [-1 0 0; 0 1 0; 0 0 -1];
Mlist_m{1} = RpToTrans([1 0 0;0 1 0;0 0 1],pJoint_st_0);
Mlist_m{2} = RpToTrans([1 0 0;0 1 0;0 0 1],pJoint_rcm_0);
Mlist_m{3} = RpToTrans(R,pJoint_tr_0);
Mlist_m{4} = RpToTrans(R, pJoint_roll_0); % for the rolling

Mlist_m{5} = RpToTrans(R,pJoint_bst_0);
Mlist_m{6} = RpToTrans(R, pJoint_flex(:,1));
Mlist_m{7} = RpToTrans(R, pJoint_flex(:,2));
Mlist_m{8} = RpToTrans(R, pJoint_flex(:,3));
Mlist_m{9} = RpToTrans(R, pJoint_flex(:,4));
Mlist_m{10} = RpToTrans(R, pJoint_flex(:,5));
Mlist_m{11} = RpToTrans(R, pJoint_flex(:,6));
Mlist_m{12} = RpToTrans(R, pJoint_flex(:,7));
Mlist_m{13} = RpToTrans(R, pJoint_flex(:,8));
Mlist_m{14} = RpToTrans(R, pJoint_flex(:,9));
Mlist_m{15} = RpToTrans(R, pJoint_flex(:,10));




