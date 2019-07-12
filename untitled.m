rosshutdown
rosinit;
P = psm('PSM1');
T_dsr  = eye(4);

q(1) = 0;
q(2) = 0;
q(3) = 0.15;
q(4) = 0;
q(5) = 0;
q(6) = pi/2;

% q(1) = 0;
% q(2) = 0;
% q(3) = 0.15;
% q(4) = 0;
% q(5) = 0;
% q(6) = 0;
goal_reached = false;
while goal_reached == false
    goal_reached = P.move_joint(q);
    pause(0.5);
end
q_home = P.get_state_joint_current();
% 