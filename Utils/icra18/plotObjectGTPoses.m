function plotObjectGTPoses(filePath,colour)

fid = fopen(filePath);
format = '%s %d %f %f %f %f %f %f';
g = textscan(fid,format,'delimiter',' ');
fclose(fid);
poses = cell2mat(g(3:end));
plot3(poses(:,1),poses(:,2),poses(:,3),'Color',colour,'Marker','o','LineStyle','none');
 
% if strcmp(filePath,'/home/mina/workspace/src/Git/do-slam/Utils/icra18/cameraGroundtruth.txt')
% for i = 1:size(poses,1)
%     iPose = poses(i,:)'; 
%     scale = 0.3;
%     plotCoordinates(iPose(1:3,1),scale*rot(iPose(4:6,1)))
% end
% 
% end

end