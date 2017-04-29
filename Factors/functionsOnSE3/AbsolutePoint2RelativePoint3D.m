function rpt = AbsolutePoint2RelativePoint3D(p,pt)

% Returns the relative homogeneous coordinates of a point expresed in 
% the frame of p in Log(SE(3)) or SE(3)

if size(p) == [4,4]
    P = p;
else
    P = ExpSE3(p);   
end

invP = invSE3(P);

if size(pt,1)==3
    rpt = invP * [pt; ones(1,size(pt,2))];
    rpt = rpt(1:3,:);
else
    rpt = invP * pt;
end
