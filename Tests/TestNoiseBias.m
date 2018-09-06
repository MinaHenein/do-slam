function [averageTr,averageR] = TestNoiseBias(n)

mu = [0,0,0,0,0,0];
sig = [0.0373, 0.001, 0.001, 0.0017, 0.0017, 0.0017];

totalTr = [0,0,0];

for i=1:n
    noise = normrnd(mu,sig,size(mu));

    x = noise(1); y = noise(2); z = noise(3);
    ox = noise(4); oy = noise(5); oz = noise(6);
    
%     angle = sqrt(ox^2+oy^2+oz^2);
%     s = sin(angle);
%     c = cos(angle);
%     A = [0 -oz  oy; 
%          oz  0 -ox;
%         -oy  ox  0 ];
      % equivalent to 
%     R = eye(3)+ (s/angle)*A+ ((1-c)/angle^2)*A^2;
    R = rot([ox;oy;oz]);    
    t = [x,y,z];

    totalTr = totalTr + t;
    rotations{i} = R;
end

averageTr = totalTr/n;
averageR = arot(rotationAveraging(rotations));

stdMean = sig/sqrt(n);

assert(3*stdMean(1) >= averageTr(1))
assert(3*stdMean(2) >= averageTr(2))
assert(3*stdMean(3) >= averageTr(3))
% assert(3*stdMean(4) >= averageR(1))
% assert(3*stdMean(5) >= averageR(2))
% assert(3*stdMean(6) >= averageR(3))

end











