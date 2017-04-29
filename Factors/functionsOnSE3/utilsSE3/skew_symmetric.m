function vx=skew_symmetric(v)
if (size(v,1)==3)&&(size(v,2)==1)
    vx=[0, -v(3,1),v(2,1);
        v(3,1), 0, -v(1,1);
        -v(2,1), v(1,1), 0];
else
    error('the input must be a 3x1 vector');
end 