function R=ypr2R(y,p,r)

% Rotation matrix for yaw pitch roll angles
% Order of rotation R=RzRyRx

 sx = sin(y); cx = cos(y);
 sy = sin(p); cy = cos(p);
 sz = sin(r); cz = cos(r);

R=[cy*cz, sx*sy*cz-cx*sz, cz*sy*cx+sx*sz;
   cy*sz, sx*sy*sz+cx*cz, cx*sy*sz-sx*cz;
   -sy,   sx*cy,          cx*cy         ];
   
   
