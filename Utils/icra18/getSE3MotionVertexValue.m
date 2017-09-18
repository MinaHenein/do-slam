function H = getSE3MotionVertexValue(filePath)

fid = fopen(filePath);
format = '%s %d %f %f %f %f %f %f';
g = textscan(fid,format,'delimiter',' ');
fclose(fid);
poses = cell2mat(g(3:end));

%poses = poses(5:7,:);

for i = 2:size(poses,1)
   relativePose = AbsoluteToRelativePoseR3xso3GlobalFrame(poses(i-1,:)',poses(i,:)');
   translations(:,i-1) = relativePose(1:3);
   rotations{i-1} = rot(relativePose(4:6));
end

R = rotationAveraging(rotations);
t = mean(translations,2);

H = [t;arot(R)];

end