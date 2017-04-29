function invT = invSE3(T)
% calculates the inverse of an SE(3) element
if (sum(size(T) ~= [4,4])) && det(T(1:3,1:3))~=1
    error('The input is not an SE(3) element')
else
R = T(1:3,1:3);
t = T(1:3,4);
invT = [R' -R'* t; 0 0 0 1];
end 


