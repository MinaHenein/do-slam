function [p2,P2] = Relative2AbsoluteSE3(p1,d)
% returns the relative position of p2 with respect to p1
% both, p1,p2 are in absolute coordinates and they are log(SE(3))

logF = 0;

if (size(p1)==[6,1]) & (size(d)==[6,1])    
    P1 = ExpSE3(p1);
    D = ExpSE3(d);
    logF = 1;
elseif (size(p1)==[4,4]) & (size(d)==[4,4]) % homogeneous coordinates 
    P1 = p1;
    D = d;
else 
    error('Unknown format. The poses should be either in log(SE(3)) or SE(3) ')
end

P2 =  P1 * D;

if logF
    p2 = LogSE3(P2);
else
    p2 = P2;
end





