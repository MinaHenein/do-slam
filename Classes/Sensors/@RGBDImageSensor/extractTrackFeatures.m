function [unique3DPoints,unique3DPointsCameras] = ...
    extractTrackFeatures(self,config,firstFrame,increment,lastFrame,method)

%--------------------------------------------------------------------------
% Author: Mina Hnenein - mina.henein@anu.edu.au - 20/06/17
% Contributors:
%--------------------------------------------------------------------------

% In the case of No GT data available:
%   - detects SIFT/SURF/FAST features in one frame,
%   - projects them ahead in following frames,
%   - looks for them in the specified region of the following frame, and
%   - keeps track of the weight of features (number of times each feature was detected)
% with the possibility to project forward for n-frames and keep constant
% number of features per frame

% In the case of GT data available:
% for the case with random features extraction:
%   - randomly selects normally distributed pixels in each frame,
%   - projects them ahead in following frames,
%   - keeps track of the weight of features (number of times each feature was detected)
% with the possibility to project forward for n-frames and keep constant
% number of features per frame

% 0- set paths for images and depth images
rgbFilePath = strcat(config.folderPath,config.sep,config.rgbImagesFolderName);
depthFilePath = strcat(config.folderPath,config.sep,config.depthImagesFolderName);
gtFilePath = strcat(config.folderPath,config.sep,config.graphFileFolderName,...
    config.sep,config.groundTruthFileName);
gtFileID = fopen(gtFilePath);
rgbFileName = config.rgbImageName;
[rgbFileNameFormat,rgbFileNameExtension] = getFileNameFormatAndExtension(rgbFileName);
depthFileName = config.depthImageName;
[depthFileNameFormat,depthFileNameExtension] = getFileNameFormatAndExtension(depthFileName);

% 1- set parameters
% sift realted parameters
    % sift threshold
siftThreshold = 0.03;
    % match threshold
minMatchScore = 0.8;
% random feature extraction related parameters
    % number of steps in image height
nStepsH = 4;
    % number of steps in image width
nStepsW = 4;
% number of frames forward to look for each feature
nFrames = 5;
% number of desired features per frame
desiredPointsPerFrame = 60;
% disp progress
print = 0;

% 2- intialise
features = [];
uniqueValid3DPoints = [];
uniqueValid3DPointsWeights= [];
uniqueValid3DPointsCameras=[];
featureCount = 0;
pointCount = 0;
framePoints = zeros((lastFrame-(firstFrame-increment))/increment,1);
KCam = self.K;

% 3- Check if data is synchronized, and synchorinze if neccessary
synchronizedData = config.synchronizedData;
if ~synchronizedData
syncedData = synchronise(config);
end

% 4- get loop closures
loopClosureFrames = detectLoopClosures(firstFrame,increment,lastFrame,...
            config,syncedData,gtFilePath);

% 5- extract features
for i=firstFrame:increment:lastFrame
    if(print)
        disp(strcat('Extracting features from frame : ', num2str(i))) ;
    end
    % extract features from frame i
    if ~isempty(syncedData)
        rgbInSyncedData = length(depthFileNameFormat)+2:length(rgbFileNameFormat)-1;
        rgbFile = strcat(syncedData(i,rgbInSyncedData),rgbFileNameExtension);
        I = imread(strcat(rgbFilePath,'/',rgbFile));
        % get depth file
        depthInSyncedData = 1:length(depthFileNameFormat);
        depthFile= strcat(syncedData(i,depthInSyncedData),depthFileNameExtension);
        depth = reshape(imread(strcat(depthFilePath,'/',depthFile)),640, 480)';
        depth = double(depth);
    else
        I = imread(strcat(rgbFilePath,'/',rgbFileName,num2str(i),rgbFileNameExtension));
        depth = reshape(load(strcat(depthFilePath,'/',depthFileName,num2str(i),...
            depthFileNameExtension)),640,480)';
    end
    [imgH,imgW,~] = size(I);
    
    switch method
        case 'sift'
            Igray = rgb2gray(I);
            [points,featureDescriptors] = sift(Igray,'Threshold',siftThreshold);
            nFeatures = size(points,2);
            for l=1:nFeatures
                featureCount = featureCount+1;
                features(featureCount,:) = [i,points(1,l),points(2,l)];
            end
        case 'surf'
            Igray = rgb2gray(I);
            points =  detectSURFFeatures(Igray);
            [featuresDescriptors, valid_points] = extractFeatures(Igray, points);
            nFeatures = size(featuresDescriptors,1);
            for l=1:nFeatures
                featureCount = featureCount+1;
                features(featureCount,:) = [i,valid_points.Location(l,1),...
                    valid_points.Location(l,2)];
            end
        case 'fast'
            Igray = rgb2gray(I);
            points =  detectFASTFeatures(Igray);
            [featuresDescriptors, valid_points] = extractFeatures(Igray, points);
            nFeatures = size(featuresDescriptors,1);
            for l=1:nFeatures
                featureCount = featureCount+1;
                features(featureCount,:) = [i,valid_points.Location(l,1),...
                    valid_points.Location(l,2)];
            end
        case 'random'
            stepH = imgH/nStepsH;
            stepW = imgW/nStepsW;
            nPts = round((desiredPointsPerFrame-...
                framePoints((i-(firstFrame-increment))/increment,1))/(nStepsH*nStepsW));
            for j=1:stepH:imgH
                for k=1:stepW:imgW
                    idx1 = randperm(nPts);
                    idx2 = randperm(nPts);
                    for l=1:nPts
                        featureCount = featureCount+1;
                        % check that feature is within image quadrant
                        if (idx2(l)+k < k+stepW && idx1(l)+j < j+stepH)
                            features(featureCount,:) = [i,idx2(l)+k,idx1(l)+j];
                        elseif (idx2(l)+k < k+stepW && idx1(l)+j > j+stepH)
                            features(featureCount,:) = [i,idx2(l)+k,min(j+stepH,imgH)];
                        elseif (idx2(l)+k > k+stepW && idx1(l)+j < j+stepH)
                            features(featureCount,:) = [i,min(k+stepW,imgW),idx1(l)+j];
                        else
                            features(featureCount,:) = [i,min(k+stepW,imgW),...
                                min(j+stepH,imgH)];
                        end
                    end
                end
            end
            frameFeatures = find(features(:,1)==i);
            nFeatures = length(frameFeatures);
            framePoints((i-(firstFrame-increment))/increment,1) = nFeatures;
    end
    
    % 6- project extracted features forward
    for j=1:nFeatures
        if(print && j==1)
            X = strcat('Projecting features extracted from frame ', num2str(i),' to 3D points');
            disp(X);
        end
        % 6.1- get 3D point
        pt = [features(frameFeatures(j),2);features(frameFeatures(j),3);1];
        PCamera = KCam \ pt;
        zCamera = depth(round(features(frameFeatures(j),3)),...
            round(features(frameFeatures(j),2)))/1000;
        if ~isempty(syncedData)
            gtInSyncedData = length(depthFileNameFormat)+length(rgbFileNameFormat)+3;
            lineScan = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines',...
                str2double(syncedData(i,gtInSyncedData:end))-1);
        else
            lineScan = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines', i-1);
        end
        cameraIDPose = str2num(cell2mat(strsplit(cell2mat(lineScan{1,1}),'')));
        fclose(gtFileID);
        cameraPose = cameraIDPose(2:end);
        cameraTranslation = cameraPose(1:3)';
        cameraRotation = quaternion2Axis([cameraPose(4);cameraPose(5);cameraPose(6);...
            cameraPose(7)]);
        cameraToWorld = [rot(cameraRotation), cameraTranslation; 0 0 0 1];
        PCamera = PCamera*zCamera;
        PWorld = cameraToWorld * [PCamera; 1];
        rgb = double(I(round(features(frameFeatures(j),3)),round(features(frameFeatures(j),2)),:));
        % Add all 3D Points of features extracted from 1st frame
        if(isempty(uniqueValid3DPoints)) 
            pointCount = pointCount+1;
            uniqueValid3DPoints(pointCount,:) = [PWorld(1:3,1)',rgb(1,1,1),rgb(1,1,2),rgb(1,1,3)];
            uniqueValid3DPointsWeights(pointCount,1) = 1;
            uniqueValid3DPointsCameras{pointCount,end+1} = i;
        else
            distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
                (uniqueValid3DPoints(:,1).'-PWorld(1,1)).^2,...
                (uniqueValid3DPoints(:,2).'-PWorld(2,1)).^2),...
                (uniqueValid3DPoints(:,3).'-PWorld(3,1)).^2))';
            [index,~] = find(distances==min(distances));
            if(norm(PWorld(1:3,1)' - uniqueValid3DPoints(index,1:3))<0.1)
                uniqueValid3DPointsWeights(index,1) = uniqueValid3DPointsWeights(index,1)+1;
                uniqueValid3DPointsCameras{index,end+1} = i;
            else
                pointCount = pointCount+1;
                uniqueValid3DPoints(pointCount,:) = [PWorld(1:3,1)',rgb(1,1,1),rgb(1,1,2),rgb(1,1,3)];
                uniqueValid3DPointsWeights(pointCount,1) = 1;
                uniqueValid3DPointsCameras{pointCount,end+1} = i;
            end
        end
        
        % 6.2- project forward in N-next frames (nFrames) to get pixel coordinate
        % get next camera pose
        if(print && j==1)
            X = strcat('Projecting 3D Points got from features extracted from frame ', num2str(i),' into next frame pixels');
            disp(X);
        end
        n = min(nFrames,(lastFrame-i)/increment);
        start = i;
        [framePoints,uniqueValid3DPointsWeights,featureCount,...
            features,uniqueValid3DPointsCameras] = projectForward(start,n,framePoints,...
            firstFrame,increment,desiredPointsPerFrame,depthFilePath,K_Cam,PWorld,...
            uniqueValid3DPoints,uniqueValid3DPointsWeights,featureCount,features,...
            uniqueValid3DPointsCameras,syncedData,gtFilePath);
                
        % 6- Loop Closures - detect pairs of frames that close the loop
        % for each 2 frames that close the loop, project forward and backward for
        % 2*nFrames in each direction (i.e start from loopClosureFrame-2*nFrames and
        % project forward for 4*nFrames frames)
        k = find(loopClosureFrames(:,1)==i,1);
        if (~isempty(k))
            [framePoints,uniqueValid3DPointsWeights,featureCount,...
                features,uniqueValid3DPointsCameras] = ...
                projectForward(loopClosureFrames(k,2)-2*n,4*n,framePoints,...
                firstFrame,increment,desiredPointsPerFrame,depthFilePath,K_Cam,PWorld,...
                uniqueValid3DPoints,uniqueValid3DPointsWeights,featureCount,features,...
                uniqueValid3DPointsCameras,syncedData,gtFilePath);
        end
    end
end

unique3DPoints = [];
unique3DPointsCameras = [];

for k = 1:size(uniqueValid3DPoints,1)
    weight = uniqueValid3DPointsWeights(k,1);
    if weight < 3
        continue
    end
    allCameras = uniqueValid3DPointsCameras(k,:);
    allCameras = allCameras(~cellfun('isempty',allCameras)); 
    allCamerasUnique = unique(cell2mat(allCameras));
    PWorld = uniqueValid3DPoints(k,:);
    if(isempty(unique3DPoints))
        unique3DPoints = [unique3DPoints; PWorld];
        for i = 1:size(allCamerasUnique,2)
           unique3DPointsCameras{size(unique3DPoints,1),i} =  allCamerasUnique(1,i); 
        end
    elseif (sum(unique3DPoints(:,1)==PWorld(1,1) &...
            unique3DPoints(:,2)==PWorld(1,2) &...
            unique3DPoints(:,3)==PWorld(1,3))==0)
        unique3DPoints = [unique3DPoints; PWorld];
        for i = 1:size(allCamerasUnique,2)
           unique3DPointsCameras{size(unique3DPoints,1),i} =  allCamerasUnique(1,i); 
        end
    end
end

if print
    disp('Features Extraction & Tracking: -- Done --');
end

end