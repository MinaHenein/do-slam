function a=ypr2axis(y,p,r)

% From yaw pitch roll to axis angles

R=ypr2R(y,p,r);
a= arot(R);
   
   
