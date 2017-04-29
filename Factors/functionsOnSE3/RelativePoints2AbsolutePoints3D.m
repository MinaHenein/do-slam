function pts = RelativePoints2AbsolutePoints3D(p,rpts)

% Returns the absolute homogeneous coordinates of points expresed in 
% the frame of p in Log(SE(3)) or SE(3)


pts = zeros(size(rpts));

for i = 1 : size (rpts,2)
    pt = RelativePoint2AbsolutePoint3D(p,rpts(:,i));
    pts (:,i) = pt;
end