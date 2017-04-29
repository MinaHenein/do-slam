function rpts = AbsolutePoints2RelativePoints3D(p,pts)

% Returns the relative homogeneous coordinates of a set of points previously expresed in 
% the absolure coordinates frames

rpts = zeros(size(pts));
for i = 1 : size (pts,2)   
    rpt = AbsolutePoint2RelativePoint3D(p,pts(:,i));
    rpts (:,i) = rpt;
end

