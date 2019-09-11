%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 06/11/2018
% plotObjectPoses_vKITTI - Sequence0001 frames 00334-00426
%--------------------------------------------------------------------------
function plotObjectPoses_vKITTI(imageRange,dataset)
% setup
if strcmp(dataset,'vkitti')
    dir = '/media/mina/Data/mina/Downloads/Virtual_KITTI/';
    cameraExtrinsicsFile = strcat(dir,'vkitti_1.3.1_extrinsicsgt/0001_clone.txt');
    objectDetectionFile = strcat(dir,'vkitti_1.3.1_motgt/0001_clone.txt');
elseif strcmp(dataset,'kitti')
    dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
    cameraExtrinsicsFile =  strcat(dir,'tracking/extrinsics/0001.txt'); 
    objectDetectionFile = strcat(dir,'tracking/data_tracking_label_2/training/label_02/0001.txt');
end

colors = {'magenta','radioactive green','leather','red','green','black','sapphire','swamp','light bluish green',...
    'butterscotch','cinnamon','chartreuse','blue'}; 
ids = [75,77,78,79,80,81,83,84,85,86,88];

fileID = fopen(cameraExtrinsicsFile,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);

for i = 1:numel(imageRange) 
    % read camera pose
    lineCell = CStr{imageRange(i)+2,1}; % +1 to skip 1st line, +1 because frames start with 00000.png
    lineArray = str2double(strsplit(lineCell,' '));
    % assert frame number
    assert(lineArray(1)==imageRange(i));
    if strcmp(dataset,'kitti')
        cameraPoseMatrix = reshape(lineArray(2:end),[4,4])';
    elseif strcmp(dataset,'vkitti')
        cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
    end
    iPose = transformationMatrixToPose(cameraPoseMatrix);

    % get all objects in frame i
    fid = fopen(objectDetectionFile);
    Data = textscan(fid,'%s','Delimiter','\n');
    cellStr = Data{1};
    fclose(fid);
    for j = 1:numel(cellStr)-1
        splitLine = strsplit(cellStr{j+1,1},' ');
        if str2double(splitLine{1,1}) == imageRange(i) && strcmp(splitLine{1,3},'Car')
            % object id
            id = str2double(splitLine{1,2});
            % object pose in camera frame
            x3d = str2double(splitLine{1,14});
            y3d = str2double(splitLine{1,15});
            z3d = str2double(splitLine{1,16});
            yaw = str2double(splitLine{1,17}); 
            if strcmp(dataset,'kitti')
                pitch = 0;
                roll = 0;
            elseif strcmp(dataset,'vkitti')
                pitch = str2double(lineCell{1,18});
                roll = str2double(lineCell{1,19});
            end
            Ry = eul2Rot([0, yaw + pi/2, 0]);
            Rx = eul2Rot([0, 0, pitch]);
            Rz = eul2Rot([roll, 0, 0]);
            R =  Ry * Rx * Rz ;
            
            objectTranslationCameraFrame = [x3d;y3d;z3d];
            objectRotationCameraFrame = R;
            objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
            % transform object pose to world frame
            disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
            id
            objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
            objectPoseWorldFrame
            scatter3(objectPoseWorldFrame(1,4),objectPoseWorldFrame(2,4),...
                objectPoseWorldFrame(3,4),'MarkerEdgeColor',rgb(colors(ids == id)),...
                'MarkerFaceColor',rgb(colors(ids == id)))
            % uncomment to show object poses as coordinate frames
%             l = 2; % coordinate axis length
%             A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
%             B = objectPoseWorldFrame*A;
%             plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',2); % x: red
%             plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',2); % y: green
%             plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',2); % z: blue
                        
            hold on
            if ismember(imageRange(i),[imageRange(1):5:imageRange(end)])
                plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6)));
            end
            view(0,0)
        end
    end
end

end