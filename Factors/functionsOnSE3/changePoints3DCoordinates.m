function rpts = changePoints3DCoordinates(p1,p2,pts)

% Returns the relative homogeneous coordinates of a point expresed in 
% the frame of p in Log(SE(3))

P = ExpSE3(p);
invP = invSE3(P);
rpts = zeros(size(pts));
for i = 1 : size (pts,2)   
    rpt = invP * pts(:,i);
    rpts (:,i) = rpt;
end