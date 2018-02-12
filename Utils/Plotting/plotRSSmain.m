
trial =  0;
for i = 1:3 % 3 different SE3MotionVertex initialization
    for j = 1:4 % 4 2points-SE3Motion edge noise levels
        trial = trial+1;
        fprintf('Trial #: %d', trial);
        plotRSS(i,j,trial);
    end
end