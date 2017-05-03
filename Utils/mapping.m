function vec = mapping(i,dim)
%maps i-D index to array of indexes in dim-D
vec = ((i-1)* dim) + 1: ((i-1)* dim) + dim;

end

