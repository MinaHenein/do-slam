function pt = RelativePoint2AbsolutePoint3D(p,rp)
%
% Returns the relative point coordinate given a point in absolute
% coordinates to a pose p in Log(SE(3)) or SE(3)

% concatenate coordinates
if size(p) == [4,4]
    P = p;
else
    P = ExpSE3(p);   
end

if size(rp,1)==3
    pt = P * [rp; ones(1,size(rp,2))];
    pt = pt(1:3,:);
else
    pt = P * rp;
end
