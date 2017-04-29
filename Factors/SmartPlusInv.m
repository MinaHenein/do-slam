function P1 = SmartPlusInv(Pose2, d)

% returns the absolute position of P2 which is at d from
% P1( in absolute)
ps2.pos=Pose2(1:3);
ps2.Q=rot(Pose2(4:6));
ds.pos=d(1:3);
ds.Q=rot(d(4:6));

ps1.q = arot(ps2.Q *(ds.Q)'); 
ps1.pos = ps2.pos - rot(ps1.q) * ds.pos;
P1=[ps1.pos;ps1.q];

end