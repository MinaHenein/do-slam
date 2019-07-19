function [frames,globalFeatures] = featureExtractionTracking(imageRange,K,rgbI,depthI,flowI,maskI,...
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
    if strcmp(settings.dataset,'kitti')
        frameName = strcat(repmat('0',1,6-numel(num2str(frameNumber))),num2str(frameNumber),'.png');
        flowFileName = strcat(repmat('0',1,6-numel(num2str(frameNumber))),num2str(frameNumber),'.flo');
    elseif strcmp(settings.dataset,'vkitti')
        frameName = strcat(repmat('0',1,5-numel(num2str(frameNumber))),num2str(frameNumber),'.png');
        
    end
    frames(i).cameraPose = transformationMatrixToPose(poseToTransformationMatrix(firstCamPose)\...
        poseToTransformationMatrix(cameraPoses(:,i)));
    if i == 1
        % extract objects
        maskIm = imread(strcat(maskI, frameName));
        frameObjects = extractFrameObjects(maskIm,motFile,frames(1),settings);
        frames(1).objects = frameObjects;
    end
    % extract features
    rgbIm = imread(strcat(rgbI,frameName));
    depthIm = imread(strcat(depthI,frameName));
    [frameFeatures,globalFeatures] = extractFrameFeatures(K,rgbIm,depthIm,frames(i).objects,...
        frames(i),nFeaturesPerFrame-size(frames(i).features.location,1),nFeaturesPerObject,globalFeatures,settings);
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
        if strcmp(settings.dataset,'kitti')
            nextFrameName = strcat(repmat('0',1,6-numel(num2str(nextFrameNumber))),num2str(nextFrameNumber),'.png');
        elseif strcmp(settings.dataset,'vkitti')
            nextFrameName = strcat(repmat('0',1,5-numel(num2str(nextFrameNumber))),num2str(nextFrameNumber),'.png');
        end
        frames(i+1).cameraPose = transformationMatrixToPose(poseToTransformationMatrix(firstCamPose)\...
        poseToTransformationMatrix(cameraPoses(:,i+1)));
        % extract objects
        nextMaskIm = imread(strcat(maskI, nextFrameName));
        nextFrameObjects = extractFrameObjects(nextMaskIm,motFile,frames(i+1),settings);
        frames(i+1).objects = nextFrameObjects;
        % project last frame features
        nextRGBIm = imread(strcat(rgbI,nextFrameName));
        nextDepthIm = imread(strcat(depthI,nextFrameName));
        if strcmp(settings.dataset,'kitti')
            flowIm = readFlowFile(strcat(flowI,flowFileName));
        elseif strcmp(settings.dataset,'vkitti')
            flowIm = '';
        end
        [nextFrameFeatures,globalFeatures] = projectFeaturesForward(frames(i),flowIm,K,...
            frames(i+1),nextRGBIm,nextDepthIm,globalFeatures,settings);
        frames(i+1).features = nextFrameFeatures;
    end
    
end





end