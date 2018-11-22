function R = roty(angle)
angle = deg2rad(angle);
R = [cos(angle) 0  sin(angle);
     0   1   0 ;
     -sin(angle)  0 cos(angle)];
 
end
