function P2 = SmartPlus(Pose1, d)

% returns the absolute position of P2 which is at d from
% P1( in absolute)
ps1.pos=Pose1(1:3);
ps1.Q=rot(Pose1(4:6));
ds.pos=d(1:3);
ds.Q=rot(d(4:6));

ps2.pos = ps1.pos + ps1.Q * ds.pos;
ps2.q = arot(ps1.Q *ds.Q); 
P2=[ps2.pos;ps2.q];

end