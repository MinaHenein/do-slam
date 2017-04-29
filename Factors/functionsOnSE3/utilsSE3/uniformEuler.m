
function [y,p,r] =  uniformEuler()
r = -2 * pi * rand() + pi;
rp = rand();
p = acos(1 - 2 * rp ) - pi/2;
y = -2 * pi * rand() + pi;
end
