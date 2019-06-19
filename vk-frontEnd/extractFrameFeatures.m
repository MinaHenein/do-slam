
function frameFeatures = extractFrameFeatures(K,rgbI,depthI,frameObjects,...
    frame,nFeaturesPerFrame,nFeaturesPerObject)

featuresLocation = [];
featuresObjectId = [];
featuresMoving = [];
featuresOriginFrame = [];
% extract features within objects' masks
for i = 1:length(frameObjects)
    objectMask = frameObjects(i).mask;
    I = rgbI.*repmat(uint8(objectMask),[1,1,3]);
    corners = detectFASTFeatures(rgb2gray(I));
    nFeaturesOnCurrentObject = sum([frame.features.objectId]==frameObjects(i).id);
    if nFeaturesPerObject - nFeaturesOnCurrentObject > 0
        strongestFeatures = corners.selectStrongest(nFeaturesPerObject - nFeaturesOnCurrentObject);
        featuresLocation = [featuresLocation; strongestFeatures.Location];
        % features on objects are assigned the object id as
        % featureObjectId 
        featuresObjectId = [featuresObjectId; repmat(frameObjects(i).id,[length(strongestFeatures),1])];
        featuresMoving = [featuresMoving; repmat(frameObjects(i).moving,[length(strongestFeatures),1])];
        featuresOriginFrame = [featuresOriginFrame; repmat(frame.number,[length(strongestFeatures),1])];
    end
end

nFeaturesToExtract = nFeaturesPerFrame - length(featuresLocation);
if nFeaturesToExtract > 0
    % features on static background are assigned -1 as featureObjectId
    featuresObjectId = [featuresObjectId;-1*ones(nFeaturesToExtract,1)];
    featuresMoving = [featuresMoving; zeros(nFeaturesToExtract,1)];
    featuresOriginFrame = [featuresOriginFrame; repmat(frame.number,[nFeaturesToExtract,1])];
    rgbICopy = rgbI;
    for i = 1:length(frameObjects)
        objectMask = frameObjects(i).mask;
        I = rgbICopy.*repmat(uint8(~objectMask),[1,1,3]);
        rgbICopy = I;
    end
    corners = detectFASTFeatures(rgb2gray(rgbICopy));
    strongestFeatures = corners.selectStrongest(nFeaturesToExtract);
    featuresLocation = [featuresLocation; strongestFeatures.Location]; 
end

frameFeatures.location = featuresLocation;
frameFeatures.moving   = featuresMoving;
frameFeatures.objectId = featuresObjectId;
frameFeatures.originFrame = featuresOriginFrame;

featuresLocation3D = zeros(3,length(frameFeatures.location));
for i=1:length(frameFeatures.location)
    pixelRow = frameFeatures.location(i,2);
    pixelCol = frameFeatures.location(i,1);
    pixelDepth = double(depthI(pixelRow,pixelCol));
    % image--> camera
    camera3DPoint = K\[pixelCol;pixelRow;1];
    camera3DPoint = camera3DPoint * pixelDepth/100;
    % camera --> world
    cameraPoseMatrix = poseToTransformationMatrix(frame.cameraPose);
    world3DPoint = cameraPoseMatrix * [camera3DPoint;1];
    featuresLocation3D(:,i) = world3DPoint(1:3);
end
frameFeatures.location3D = featuresLocation3D;
end