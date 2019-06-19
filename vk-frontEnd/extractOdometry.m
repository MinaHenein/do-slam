function odometry = extractOdometry(cameraPoses)

nPoses = size(cameraPoses,2);
odometry = zeros(6,nPoses-1);
for i=1:nPoses-1
    odometry(:,i) = AbsoluteToRelativePoseR3xso3(cameraPoses(:,i),cameraPoses(:,i+1));
end

end