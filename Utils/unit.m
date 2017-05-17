function [n] = unit(v)
%UNIT scales vector v to unit norm
if norm(v) == 0
    n = v;
else
    n = v/norm(v);
end

end
