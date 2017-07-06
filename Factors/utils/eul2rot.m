%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 06/07/2017
% Contributors:
%--------------------------------------------------------------------------

function rot = eul2rot(angles)
%EUL2ROT Converts euler angle representation to rotation matrix.  Set to
%default sequence of z, y, x.
% http://web.mit.edu/2.05/www/Handout/HO2.PDF

a = angles(1); % alpha
b = angles(2); % beta
g = angles(3); % gamma

T1 = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0 0 1];
T2 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
T3 = [1 0 0; 0 cos(g) -sin(g); 0 sin(g) cos(g)];

rot = T1*T2*T3;

end

