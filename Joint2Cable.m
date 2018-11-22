function [l4ml2,l3ml1] = Joint2Cable(phi,theta)
D = 5;
d = 2.5;

l3ml1   = 2*theta*d*cos(phi);
l4ml2   = 2*theta*d*sin(phi);



end