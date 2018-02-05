trial =  0;
for i = 1 % 2 different SE3MotionVertex initialization
    for j = 2 % 3 2points-SE3Motion edge noise levels
        trial = trial+1;
        fprintf('Trial #: %d',trial);
        Test12_SE3MotionNObservationsNeeded2(i,j,trial);
%         plotTest12_trajectories;
    end
end