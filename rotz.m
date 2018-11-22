function R = rotz(angle)
angle = deg2rad(angle);
R = [cos(angle) -sin(angle) 0 ; 
     sin(angle) cos(angle) 0;
     0  0 1];
 
end
