
name = 'test13';
for i = 1:3 % 3 different SE3MotionVertex initialization
    for j = 1:4 % 4 2points-SE3Motion edge noise levels
        plotRSS(i,j,name);
    end
end