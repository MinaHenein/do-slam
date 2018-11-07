%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/11/2018
% plotObjectPoses_vKITTI - Sequence0001 frames 00334-00426
%--------------------------------------------------------------------------
function plotObjectPoses_vKITTI(imageRange)
% setup
dir = '/home/mina/Downloads/vKitti/';
cameraExtrinsicsFile = strcat(dir,'vkitti_1.3.1_extrinsicsgt/0001_clone.txt');
objectDetectionFile = strcat(dir,'vkitti_1.3.1_motgt/0001_clone.txt');

colors = {'red','green','blue','black','leather','swamp','plum',...
    'dark red','magenta','sapphire'};
ids = [75,78,79,80,81,83,84,85,86,88];
for i = imageRange 
    % read camera pose
    fid = fopen(cameraExtrinsicsFile);
    lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+1);
    fclose(fid);
    lineArray = str2num(cell2mat(lineCell{1,1}));
    assert(lineArray(1)==i);
    cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
    % get all objects in frame i
    fid = fopen(objectDetectionFile);
    Data = textscan(fid,'%s','Delimiter','\n');
    cellStr = Data{1};
    fclose(fid);
    for j = 2:length(cellStr)
        splitLine = strsplit(cellStr{j,1},' ');
        if str2double(splitLine{1,1}) == i && strcmp(splitLine{1,3},'Car')
            % object id
            id = str2double(splitLine{1,2});
            % object pose in camera frame
            x3d = str2double(splitLine{1,14});
            y3d = str2double(splitLine{1,15});
            z3d = str2double(splitLine{1,16});
            yaw = str2double(splitLine{1,17});
            pitch = str2double(splitLine{1,18});
            roll = str2double(splitLine{1,19});
            objectTranslationCameraFrame = [x3d;y3d;z3d];
            objectRotationCameraFrame = angle2dcm(yaw,pitch,roll);
            objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
            % transform object pose to world frame
            objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
            scatter3(objectPoseWorldFrame(1,4),objectPoseWorldFrame(2,4),...
                objectPoseWorldFrame(3,4),'MarkerEdgeColor',rgb(colors(ids == id)),...
                'MarkerFaceColor',rgb(colors(ids == id)))
            hold on
        end
    end
end

end