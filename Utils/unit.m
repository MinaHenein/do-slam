function [n] = unit(v)

if norm(v) == 0
    n = v;
else
    n = v/norm(v);
end

end %function
