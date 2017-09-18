function plotObjectPoses(filePath,H,colour)

fid = fopen(filePath);
format = '%s %d %f %f %f %f %f %f';
g = textscan(fid,format,'delimiter',' ');
fclose(fid);
poses = cell2mat(g(3:end));
posesN(:,1) = poses(1,:);

for i=2:size(poses,1)
    posesN(:,i) = RelativeToAbsolutePoseR3xso3GlobalFrame(posesN(:,i-1),H);
end

plot3(posesN(1,:),posesN(2,:),posesN(3,:),'Color',colour,'Marker','o','LineStyle','none');

end