function [R, aa] = attitude3D(p1,p2)
%computes the attitude of the segment defined by p1 and p2
% the origine is p1 
% use http://math.stackexchange.com/questions/748144/how-to-find-the-rotation-matrix-that-will-align-an-arbitrary-vector-to-an-axis

d = p2 - p1;
d = norm3Dvect(d);

c = dot([1 0 0],d);
a =  cross([1 0 0],d)'; % axis of rotation
s = norm(a);
ax = skew_symmetric(a);
if s == 0
    R = eye(3);
else
    R = eye(3) + ax + ax^2 * (1 - c)/(s^2);
end

aa = arot(R);

end

function vec_n = norm3Dvect(vec)
norm_vec = norm(vec);
if (norm_vec <= 0)
    vec_n = zeros(size(vec));
else
    vec_n = vec ./ norm_vec;
end
end
