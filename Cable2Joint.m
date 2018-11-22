function [phi,theta] = Cable2Joint(l4ml2,l3ml1)
D   = 5; % mm

phi = atan2(l4ml2,l3ml1); % checked
theta  = sqrt(l4ml2^2+l3ml1^2)/D;
% thetab_old = 2*asin( sqrt(l3ml1^2+l4ml2^2) /m.N/m.D/1000);
% thetab_con = 2*asin(l3ml1/m.N/m.D/cos(phi)/1000);


end