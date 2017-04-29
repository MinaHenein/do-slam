function d = SmartMinus(Pose1, Pose2)

% Implements Pose2-Pose1 on manifold
% both, Pose1 & Pose2 are in absolute 
% the same as A2R
ps1.pos=Pose1(1:3);
ps1.Q=rot(Pose1(4:6));
ps2.pos=Pose2(1:3);
ps2.Q=rot(Pose2(4:6));

inv_ps1.Q = (ps1.Q)';
ds.pos = inv_ps1.Q * (ps2.pos - ps1.pos);
ds.axis = arot(inv_ps1.Q * ps2.Q);
d=[ds.pos;ds.axis];


end
