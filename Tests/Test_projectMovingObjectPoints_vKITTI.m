%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 02/11/2018
% Test_projectMovingObjectPoint_vKITTI - Sequence0001 frames 00334-00426
%--------------------------------------------------------------------------
% setup
dir = '/home/mina/Downloads/vKitti/';
display = 1;
i = randi([334 426],1); %random image index
cameraExtrinsicsFile = strcat(dir,'vkitti_1.3.1_extrinsicsgt/0001_clone.txt');
segmentationEncodingFile = strcat(dir,'vkitti_1.3.1_scenegt/0001_clone_scenegt_rgb_encoding.txt');
segmentationGTFile = strcat(dir,'/vkitti_1.3.1_scenegt/0001/clone/');
objectDetectionFile = strcat(dir,'vkitti_1.3.1_motgt/0001_clone.txt');
maskRCNNCentroidFile = strcat(dir(1:end-1),'_MASK-RCNN/classCentroid.txt');
% camera intrinsics
K = [725,   0,     620.5;
    0,    725,     187.0;
    0,      0,        1];
% read rgb image
rgbI = imread(strcat(dir,'rgbImages/00',num2str(i),'.png'));
% read depth image
depthI = imread(strcat(dir,'depthImages/00',num2str(i),'.png'));
% read segmentation image
segmentationI = imread(strcat(segmentationGTFile,'00',num2str(i),'.png'));
% n- features to track
nFeatures = 1;
% get centroids of all objects in frame
fid = fopen(maskRCNNCentroidFile);
Data = textscan(fid,'%s','Delimiter','\n');
cellStr = Data{1};
fclose(fid);
cellIndex = strfind(cellStr,strcat('00',num2str(i),'.png'));
Index = find(~cellfun('isempty', cellIndex));
% randomly select one detected object
object = '';
while ~strcmp(object,'car') 
    j= randi([1 length(Index)],1);
    fid = fopen(maskRCNNCentroidFile);
    line = textscan(fid,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    fclose(fid);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    object = splitLine{1,2};
end
% get object centroid
centroidX = str2double(splitLine{1,3});
centroidY = str2double(splitLine{1,4});
% get object centroid RGB value
objectRGB = [segmentationI(centroidY,centroidX,1),segmentationI(centroidY,centroidX,2),...
    segmentationI(centroidY,centroidX,3)];
% get class and label
[~,classGT] = getObjectClass(segmentationEncodingFile, objectRGB);
% get object bounding box
boundingBox = getObjectBoundingBox(objectDetectionFile,i,classGT);
% project n-randomly selected features on object j onto next frame
features = [randi([boundingBox(2) boundingBox(4)],nFeatures,1),...
    randi([boundingBox(1) boundingBox(3)],nFeatures,1)];
if display
    figure
    imshow(rgbI)
    hold on
    scatter(centroidX,centroidY,'filled','bo','LineWidth',10);
    hold on
    plot([boundingBox(1),boundingBox(1)],[boundingBox(2),boundingBox(4)],'r-','LineWidth',2)
    hold on
    plot([boundingBox(1),boundingBox(3)],[boundingBox(4),boundingBox(4)],'r-','LineWidth',2)
    hold on
    plot([boundingBox(3),boundingBox(3)],[boundingBox(4),boundingBox(2)],'r-','LineWidth',2)
    hold on
    plot([boundingBox(3),boundingBox(1)],[boundingBox(2),boundingBox(2)],'r-','LineWidth',2)
    hold on
    scatter(features(:,2),features(:,1),'rx','LineWidth',2)
    hold off
end

% read camera pose
fid = fopen(cameraExtrinsicsFile);
lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+1);
fclose(fid);
lineArray = str2num(cell2mat(lineCell{1,1}));
assert(lineArray(1)==i);
cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
fid = fopen(cameraExtrinsicsFile);
lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+2);
fclose(fid);
lineArray = str2num(cell2mat(lineCell{1,1}));
assert(lineArray(1)==i+1);
nextCameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');

nextFrameFeatures = zeros(nFeatures,2);
for k=1:size(features,1)
    % get world 3D point
    pixelRow = features(k,1);
    pixelCol = features(k,2);
    camera3DPoint = K\[pixelCol;pixelRow;1];
    camera3DPoint = camera3DPoint* double(depthI(pixelRow,pixelCol))/100;
    world3DPoint = cameraPoseMatrix * [camera3DPoint;1];
    % get object pose in last camera frame
    fid = fopen(objectDetectionFile);
    Data = textscan(fid,'%s','delimiter','\n','whitespace',' ');
    cellData = Data{1};
    cellIndex = strfind(cellData, strcat({num2str(i)},{' '},{classGT(5:end)},{' '},{'Car'}));
    fclose(fid);
    lineIndex = find(~cellfun('isempty', cellIndex));
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
    % extract object pose in current camera frame
    fid = fopen(objectDetectionFile);
    Data = textscan(fid,'%s','delimiter','\n','whitespace',' ');
    cellData = Data{1};
    cellIndex = strfind(cellData, strcat({num2str(i+1)},{' '},{classGT(5:end)},{' '},{'Car'}));
    fclose(fid);
    lineIndex = find(~cellfun('isempty', cellIndex));
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
    nextObjectTranslationCameraFrame = [x3d;y3d;z3d];
    nextObjectRotationCameraFrame = angle2dcm(yaw,pitch,roll);
    nextObjectPoseCameraFrame = [nextObjectRotationCameraFrame, nextObjectTranslationCameraFrame; 0 0 0 1];
    nextObjectPoseWorldFrame =  nextCameraPoseMatrix * nextObjectPoseCameraFrame;
    % object motion in object Frame
    objectMotionObjectFrame =  objectPoseWorldFrame \ nextObjectPoseWorldFrame;
    % object motion in world frame
    objectMotionWorldFrame = objectPoseWorldFrame * objectMotionObjectFrame...
        / objectPoseWorldFrame;
    % get new 3D point
    if objectRGB(1) ~= segmentationI(pixelRow,pixelCol,1)  || ...
        objectRGB(2) ~= segmentationI(pixelRow,pixelCol,2) || ...
        objectRGB(3) ~= segmentationI(pixelRow,pixelCol,3)
        continue
    end
    movedWorld3DPoint = objectMotionWorldFrame * world3DPoint;
    % project onto next frame
    nextCamera3DPoint = nextCameraPoseMatrix \ movedWorld3DPoint;
    nextCamera3DPoint = nextCamera3DPoint(1:3,1);
    % camera --> image
    nextImagePoint = K * nextCamera3DPoint;
    nextImagePoint = nextImagePoint/nextImagePoint(3);
    nextFrameFeatures(k,:) = round(nextImagePoint(1:2,1)'); 
end

if display
    nextrgbI = imread(strcat(dir,'rgbImages/00',num2str(i+1),'.png'));
    figure
    imshow(nextrgbI)
    hold on
    scatter(nextFrameFeatures(:,1),nextFrameFeatures(:,2),'rx','LineWidth',2)
    hold off
end
