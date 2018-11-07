%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 07/11/18
% Contributors:
%--------------------------------------------------------------------------
% TEST_VKITTICONSTANTOBJECTMOTION
% Testing if the object motion of the objects in vKitti is constant
%--------------------------------------------------------------------------
dir = '/home/mina/Downloads/vKitti/';
cameraExtrinsicsFile = strcat(dir,'vkitti_1.3.1_extrinsicsgt/0001_clone.txt');
objectDetectionFile = strcat(dir,'vkitti_1.3.1_motgt/0001_clone.txt');
imageRange = 334:426;
colors = {'red','green','blue','black','leather','swamp','plum',...
    'dark red','magenta','sapphire'};
ids = [75,78,79,80,81,83,84,85,86,88];

testingVS = 'averageMotion'; %'first2FramesMotion';

if strcmp(testingVS, 'first2FramesMotion')
    % testing vs motion from frame 1 --> 2 applied to every pose
    count = zeros(length(ids),1);
    for i=1:length(ids)
        figure;
        for j = imageRange
            % read camera pose
            fid = fopen(cameraExtrinsicsFile);
            lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',j+1);
            fclose(fid);
            lineArray = str2num(cell2mat(lineCell{1,1}));
            assert(lineArray(1)==j);
            cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
            % get object i pose in frame j
            fid = fopen(objectDetectionFile);
            Data = textscan(fid,'%s','Delimiter','\n');
            cellStr = Data{1};
            fclose(fid);
            cellIndex = strfind(cellStr,strcat({num2str(j)},{' '},{num2str(ids(i))},{' '},'Car'));
            lineIndex = find(~cellfun('isempty', cellIndex));
            if ~isempty(lineIndex)
                count(i,1)= count(i,1)+1;
                fid = fopen(objectDetectionFile);
                line = textscan(fid,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                fclose(fid);
                line = cell2mat(line{1,1});
                splitLine = str2double(strsplit(line,' '));
                x3d = splitLine(14);
                y3d = splitLine(15);
                z3d = splitLine(16);
                yaw = splitLine(17);
                pitch = splitLine(18);
                roll = splitLine(19);
                objectTranslationCameraFrame = [x3d;y3d;z3d];
                objectRotationCameraFrame = angle2dcm(yaw,pitch,roll);
                objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
                % transform object pose to world frame
                objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
                if count(i,1) == 1
                    firstObjectPoseWorldFrame = objectPoseWorldFrame;
                    % initially object pose from motion = first object pose
                    lastObjectPoseFromMotion = firstObjectPoseWorldFrame;
                end
                if count(i,1) == 2
                    secondObjectPoseWorldFrame = objectPoseWorldFrame;
                    % object motion from first 2 poses
                    objectMotion = firstObjectPoseWorldFrame \ secondObjectPoseWorldFrame;
                end
                if count(i,1)>=2
                    % motion is in body-fixed frame
                    objectPoseFromMotion = lastObjectPoseFromMotion * objectMotion;
                    scatter3(objectPoseFromMotion(1,4),objectPoseFromMotion(2,4),...
                        objectPoseFromMotion(3,4),'r+');
                    hold on
                    scatter3(objectPoseWorldFrame(1,4),objectPoseWorldFrame(2,4),...
                        objectPoseWorldFrame(3,4),'bo');
                    lastObjectPoseFromMotion = objectPoseFromMotion;
                end
            end
        end
    end
    
elseif strcmp(testingVS,'averageMotion')
    
    % testing vs average motion applied to every pose
    count = zeros(length(ids),1);
    averageObjectMotion = zeros(6,length(ids));
    for i=1:length(ids)
        rotations = {};
        translations = [];
        for j = imageRange
            % read camera pose
            fid = fopen(cameraExtrinsicsFile);
            lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',j+1);
            fclose(fid);
            lineArray = str2num(cell2mat(lineCell{1,1}));
            assert(lineArray(1)==j);
            cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
            % get object i pose in frame j
            fid = fopen(objectDetectionFile);
            Data = textscan(fid,'%s','Delimiter','\n');
            cellStr = Data{1};
            fclose(fid);
            cellIndex = strfind(cellStr,strcat({num2str(j)},{' '},{num2str(ids(i))},{' '},'Car'));
            lineIndex = find(~cellfun('isempty', cellIndex));
            if ~isempty(lineIndex)
                count(i,1)= count(i,1)+1;
                fid = fopen(objectDetectionFile);
                line = textscan(fid,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                fclose(fid);
                line = cell2mat(line{1,1});
                splitLine = str2double(strsplit(line,' '));
                x3d = splitLine(14);
                y3d = splitLine(15);
                z3d = splitLine(16);
                yaw = splitLine(17);
                pitch = splitLine(18);
                roll = splitLine(19);
                objectTranslationCameraFrame = [x3d;y3d;z3d];
                objectRotationCameraFrame = angle2dcm(yaw,pitch,roll);
                objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
                % transform object pose to world frame
                objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
                if count(i,1)>= 2
                    objectMotion = objectPoseWorldFrame/lastObjectPoseWorldFrame;
                    rotM = objectMotion(1:3,1:3);
                    t = objectMotion(1:3,4);
                    rotations{count(i,1)-1} = rotM;
                    translations(:,count(i,1)-1) = t;
                end
                lastObjectPoseWorldFrame = objectPoseWorldFrame;
            end
        end
        R = rotationAveraging(rotations);
        t = mean(translations,2);
        averageObjectMotion(:,i) = [t;arot(R)];
    end
    
    count = zeros(length(ids),1);
    for i=1:length(ids)
        objectMotion = poseToTransformationMatrix(averageObjectMotion(:,i));
        figure;
        for j = imageRange
            % read camera pose
            fid = fopen(cameraExtrinsicsFile);
            lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',j+1);
            fclose(fid);
            lineArray = str2num(cell2mat(lineCell{1,1}));
            assert(lineArray(1)==j);
            cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
            % get object i pose in frame j
            fid = fopen(objectDetectionFile);
            Data = textscan(fid,'%s','Delimiter','\n');
            cellStr = Data{1};
            fclose(fid);
            cellIndex = strfind(cellStr,strcat({num2str(j)},{' '},{num2str(ids(i))},{' '},'Car'));
            lineIndex = find(~cellfun('isempty', cellIndex));
            if ~isempty(lineIndex)
                count(i,1)= count(i,1)+1;
                fid = fopen(objectDetectionFile);
                line = textscan(fid,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                fclose(fid);
                line = cell2mat(line{1,1});
                splitLine = str2double(strsplit(line,' '));
                x3d = splitLine(14);
                y3d = splitLine(15);
                z3d = splitLine(16);
                yaw = splitLine(17);
                pitch = splitLine(18);
                roll = splitLine(19);
                objectTranslationCameraFrame = [x3d;y3d;z3d];
                objectRotationCameraFrame = angle2dcm(yaw,pitch,roll);
                objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
                % transform object pose to world frame
                objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
                if count(i,1) == 1
                    % initially object pose from motion = first object pose
                    lastObjectPoseFromMotion = objectPoseWorldFrame;
                end
                if count(i,1)>=2
                    % motion is in body-fixed frame
                    objectPoseFromMotion = objectMotion * lastObjectPoseFromMotion;
                    scatter3(objectPoseFromMotion(1,4),objectPoseFromMotion(2,4),...
                        objectPoseFromMotion(3,4),'r+');
                    hold on
                    scatter3(objectPoseWorldFrame(1,4),objectPoseWorldFrame(2,4),...
                        objectPoseWorldFrame(3,4),'bo');
                    lastObjectPoseFromMotion = objectPoseFromMotion;
                end
            end
        end
    end
end