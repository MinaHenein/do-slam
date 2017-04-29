function d = Absolute2RelativeSE3(p1,p2)
% returns the relative position of p2 with respect to p1, p1 is the origin
% both, p1,p2 are in absolute coordinates and they are log(SE(3)) or SE(3)

logF = 0;

if (size(p1)==[6,1]) & (size(p2)==[6,1])    
    P1 = ExpSE3(p1);
    P2 = ExpSE3(p2);
    logF = 1;
elseif (size(p1)==[4,4]) & (size(p2)==[4,4]) % homogeneous coordinates 
    P1 = p1;
    P2 = p2;
else 
    error('Unknown format. The poses should be either in log(SE(3)) or SE(3) ')
end

D = invSE3(P1) * P2;

if logF
    d = LogSE3(D);
else
    d = D;
end



