function R = rotationAveraging(Rotations)

err = 1e-5.*ones(3,3);
R   = Rotations{1};

Temp=0;
for i=1:length(Rotations)
    
    theta = acos((trace(R'.*Rotations{i})-1)/2);
    if theta == 0
        logR = 0;
    else
        logR = (theta/2*sin(theta))*(R'.*Rotations{i}-(R'.*Rotations{i})');
    end
    Temp=logR+Temp;
end
r=(1/length(Rotations)).*Temp;

while (abs(r))>err
    
    if (abs(r))<err
        return;
    else R=R.*exp(r);
    end
    
    Temp=0;
    for i=1:length(Rotations)
        Temp=logR + Temp;
    end
    r=(1/length(Rotations)).*Temp;
    
end

end