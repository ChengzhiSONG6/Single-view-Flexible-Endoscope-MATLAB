function [q1,q2]=T(uu)
       
       cx=640;
       cy=360;
       f=707/2;
       x_round=uu(1);
       y_round2=uu(2);
       y_round=-(x_round-cx);
       x_round=y_round2-cy;
       q1=atand(sqrt(y_round^2+x_round^2)/f)*3.14/180;
%        if x_round<0
%        q1=atand(sqrt(y_round^2+x_round^2)/f)*3.14/180;
%        
%        end
%        
%        if x_round>=0
%            q1=-atand(sqrt(y_round^2+x_round^2)/f)*3.14/180;
%        end
       
       if y_round>0 & x_round>0
       q2=abs(atand(y_round/x_round))*3.14/180;
       end
       if y_round>0 & x_round<0
       q2=(abs(atand(x_round/y_round))+90)*3.14/180;
       end
       if y_round<0 & x_round<0
       q2=-(abs(atand(x_round/y_round))+90)*3.14/180;
       end
       if y_round<0 & x_round>0
       q2=-(abs(atand(y_round/x_round)))*3.14/180;
       end
       
       if y_round==0 & x_round>=0
           q2=0;
       end
      if y_round<0 & x_round==0
           q2=3.14/2;
      end
      if x_round<0 & y_round==0
           q2=3.14;
      end
      if x_round==0 & y_round>0
           q2=-3.14/2;
       end
if q2==0
   q2=0.01; 
end

if q1==0
   q1=0.01; 
end
   

    
end