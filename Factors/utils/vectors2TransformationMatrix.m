%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 11/08/17
% Contributors:
%--------------------------------------------------------------------------
% returns the transfomation matrix T that would transform v2 to v1
% v1: vector in world coordinates
% v2: vector in object cooridantes

function T = vectors2TransformationMatrix(v1,v2)

%check length of v1, v2
if size(v1,1)== 3
    v1 = [v1;1];
elseif size(v1,1)== 4
else
    error('wrong vector size')
end
if size(v2,1)== 3
    v2 = [v2;1];
elseif size(v2,1)== 4
else
    error('wrong vector size')
end


R1 = vrrotvec2mat(vrrotvec(v2(1:3),v1(1:3)));
t1 = [0;0;0];
T1 =[R1,t1;0 0 0 1];

v1copy = T1*v2;
t2 = v1-v1copy;
R2 = R1;
T =[R2,t2(1:3);0 0 0 1];

v1copy = T*v2;

assert(v1copy(1)-v1(1)<1e-5)
assert(v1copy(2)-v1(2)<1e-5)
assert(v1copy(3)-v1(3)<1e-5)

end
