function [frames,globalFeatures] = featureExtractionTracking(imageRange,K,rgbI,depthI,maskI,...
    motFile,cameraPoses,nFeaturesPerFrame,nFeaturesPerObject,maxBackgroundFeaturesPerFrame,settings)

% setup 
frames(1).features.location = [];
frames(1).features.objectId = [];
% global features
globalFeatures.location3D = [];
globalFeatures.weight = [];
globalFeatures.id = [];
globalFeatures.frame = [];
globalFeatures.cameraLocation = [];
globalFeatures.static = [];
globalFeatures.objectId = [];
% dynamic features associations
globalFeatures.dynamicAssociation = {};

firstCamPose = cameraPoses(:,1);

for i = 1:numel(imageRange)
    pbar(i,numel(imageRange),[],[num2str(i),'/',num2str(numel(imageRange))],'off');
    
    % frame i
    frameNumber = imageRange(i);
    frames(i).number = imageRange(i);
    frameName = strcat(repmat('0',1,5-numel(num2str(frameNumber))),num2str(frameNumber),'.png');
    frames(i).cameraPose = transformationMatrixToPose(poseToTransformationMatrix(firstCamPose)\...
        poseToTransformationMatrix(cameraPoses(:,i)));
    if i == 1
        % extract objects
        maskIm = imread(strcat(maskI, frameName));
        frameObjects = extractFrameObjects(maskIm,motFile,frames(1));
        frames(1).objects = frameObjects;
    end
    % extract features
    rgbIm = imread(strcat(rgbI,frameName));
    depthIm = imread(strcat(depthI,frameName));
    [frameFeatures,globalFeatures] = extractFrameFeatures(K,rgbIm,depthIm,frames(i).objects,...
        frames(i),nFeaturesPerFrame-size(frames(i).features.location,1),nFeaturesPerObject,globalFeatures);
    if isempty(frames(i).features.location)
        frames(i).features = frameFeatures;
    else
        if ~isempty(frameFeatures.location)
            frames(i).features.location = [frames(i).features.location; frameFeatures.location];
            frames(i).features.moving = [frames(i).features.moving; frameFeatures.moving];
            frames(i).features.objectId = [frames(i).features.objectId; frameFeatures.objectId];
            frames(i).features.location3D = [frames(i).features.location3D, frameFeatures.location3D];
            frames(i).features.originFrame = [frames(i).features.originFrame; frameFeatures.originFrame];
            frames(i).features.id = [frames(i).features.id; frameFeatures.id];
        end
    end
    
%     % postprocess frames to limit number of background features
%     frame = postProcessFrameFeatures(frames(i),maxBackgroundFeaturesPerFrame);
%     frames(i) = frame;
    
    if i < numel(imageRange)
        % frame i+1
        nextFrameNumber = imageRange(i+1);
        frames(i+1).number = imageRange(i+1);
        nextFrameName = strcat(repmat('0',1,5-numel(num2str(nextFrameNumber))),num2str(nextFrameNumber),'.png');
        frames(i+1).cameraPose = transformationMatrixToPose(poseToTransformationMatrix(firstCamPose)\...
        poseToTransformationMatrix(cameraPoses(:,i+1)));
        % extract objects
        nextMaskIm = imread(strcat(maskI, nextFrameName));
        nextFrameObjects = extractFrameObjects(nextMaskIm,motFile,frames(i+1));
        frames(i+1).objects = nextFrameObjects;
        % project last frame features
        nextRGBIm = imread(strcat(rgbI,nextFrameName));
        [nextFrameFeatures,globalFeatures] = projectFeaturesForward(frames(i),K,...
            frames(i+1),nextRGBIm,globalFeatures,settings);
        frames(i+1).features = nextFrameFeatures;
    end
    
end





end