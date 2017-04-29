function axis = arot(Q)
% AROT determine scaled axis represenation from 3x3 rotation matrix Q.
% http://en.wikipedia.org/wiki/Logarithm_of_a_matrix
    if size(Q) ~= [3 3]
        error('size mismatch')
    end
    
    c = (trace(Q) - 1) / 2.;
    if c < -1
        c = -1;
    elseif c > 1
        c = 1;
    end
       
    angle = acos(c);
    
    if (angle ~= 0)
        % normalize axis
        factor = angle / (2 * sin(angle));
    else
        % in the limit
        factor = 1/2.;
    end
    
    axis = factor * [Q(3,2) - Q(2,3);
                     Q(1,3) - Q(3,1);
                     Q(2,1) - Q(1,2)];
end
