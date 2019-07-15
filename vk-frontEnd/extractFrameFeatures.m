function [frameFeatures,globalFeatures] = extractFrameFeatures(K,rgbI,depthI,frameObjects,...
    frame,nFeaturesPerFrame,nFeaturesPerObject,globalFeatures)

featuresLocation = [];
featuresObjectId = [];
featuresMoving = [];
featuresOriginFrame = [];
featuresId = [];

% extract features within objects' masks
for i = 1:length(frameObjects)
    objectMask = frameObjects(i).mask;
    shrankObjectMask = bwmorph(objectMask,'thin',6);
    I = rgbI.*repmat(uint8(shrankObjectMask),[1,1,3]);
    corners = detectFASTFeatures(rgb2gray(I));
    nFeaturesOnCurrentObject = sum([frame.features.objectId]==frameObjects(i).id);
    if nFeaturesPerObject - nFeaturesOnCurrentObject > 0
        strongestFeatures = corners.selectStrongest(nFeaturesPerObject - nFeaturesOnCurrentObject);
        featuresLocation = [featuresLocation; strongestFeatures.Location];
        nFeatures = length(strongestFeatures);
        % features on objects are assigned the object id as
        % featureObjectId 
        featuresObjectId = [featuresObjectId; repmat(frameObjects(i).id,[length(strongestFeatures),1])];
        featuresMoving = [featuresMoving; repmat(frameObjects(i).moving,[length(strongestFeatures),1])];
        featuresOriginFrame = [featuresOriginFrame; repmat(frame.number,[length(strongestFeatures),1])];
        featuresId = [featuresId; [length(globalFeatures.location3D)+1:length(globalFeatures.location3D) + ...
            length(strongestFeatures)]'];
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
        thickenedObjectMask = bwmorph(objectMask,'thicken',6);
        I = rgbICopy.*repmat(uint8(~thickenedObjectMask),[1,1,3]);
        rgbICopy = I;
    end
    corners = detectFASTFeatures(rgb2gray(rgbICopy));
    strongestFeatures = corners.selectStrongest(nFeaturesToExtract);
    featuresLocation = [featuresLocation; strongestFeatures.Location];
    featuresId = [featuresId; [length(globalFeatures.location3D)+1:length(globalFeatures.location3D) + ...
        length(strongestFeatures)]']; 
end

frameFeatures.location = featuresLocation;
frameFeatures.moving   = featuresMoving;
frameFeatures.objectId = featuresObjectId;
frameFeatures.originFrame = featuresOriginFrame;
frameFeatures.id = featuresId;

globalLocation3D = globalFeatures.location3D;
globalWeight = globalFeatures.weight;
globalId = globalFeatures.id;
globalFrame = globalFeatures.frame;
globalCameraLocation = globalFeatures.cameraLocation;
globalStatic = globalFeatures.static;
globalObjectId = globalFeatures.objectId;
globalAssociation = globalFeatures.dynamicAssociation;

featuresLocation3D = zeros(3,length(frameFeatures.location));
for i=1:size(frameFeatures.location,1)
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
    % global features
    if frameFeatures.moving(i) == 0
        if isempty(globalLocation3D)
            globalLocation3D(:,1) = world3DPoint(1:3,1);
            globalWeight(1,1) = 1;
            globalId(1,1) = frameFeatures.id(i);
            globalFrame(1,1) = frameFeatures.originFrame(i);
            globalCameraLocation(:,1) = camera3DPoint;
            globalStatic(1,1) = 1;
            globalObjectId(1,1) = -1;
        else
            % compute distance to all 3D points
            distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
                (globalLocation3D(1,:).'-world3DPoint(1,1)).^2,...
                (globalLocation3D(2,:).'-world3DPoint(2,1)).^2),...
                (globalLocation3D(3,:).'-world3DPoint(3,1)).^2))';
            % find min distant point
            [~,index] = find(distances == min(distances));
            % if closest point is within 1 mm in 3D
            if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) == 0)
                globalWeight(index,1) = globalWeight(index,1) + 1;
            else
                globalLocation3D = [globalLocation3D, world3DPoint(1:3)];
                globalWeight(end+1,1) = 1;
                globalId(end+1,1) = size(globalLocation3D,2);
                globalFrame(end+1,1) = frameFeatures.originFrame(i);
                globalCameraLocation = [globalCameraLocation, camera3DPoint(1:3)];
                globalStatic(end+1,1) = 1;
                globalObjectId(end+1,1) = -1;
            end
        end
    else
        % dynamic point
        if isempty(globalLocation3D)
            globalLocation3D(:,1) = world3DPoint(1:3,1);
            globalWeight(1,1) = 1;
            globalId(1,1) = frameFeatures.id(i);
            globalFrame(1,1) = frameFeatures.originFrame(i);
            globalCameraLocation(:,1) = camera3DPoint;
            globalStatic(1,1) = 0;
            globalObjectId(1,1) = frameFeatures.objectId(i);
        else
            % compute distance to all 3D points
            distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
                (globalLocation3D(1,:).'-world3DPoint(1,1)).^2,...
                (globalLocation3D(2,:).'-world3DPoint(2,1)).^2),...
                (globalLocation3D(3,:).'-world3DPoint(3,1)).^2))';
            % find min distant point
            [~,index] = find(distances == min(distances));
            % if closest point is within 1 mm in 3D
            if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) == 0)
                assert(globalStatic(index,1) == 0);
                assert(globalObjectId(index,1) == frameFeatures.objectId(i));
            else
                globalLocation3D = [globalLocation3D, world3DPoint(1:3)];
                globalWeight(end+1,1) = 1;
                globalId(end+1,1) = size(globalLocation3D,2);
                globalFrame(end+1,1) = frameFeatures.originFrame(i);
                globalCameraLocation = [globalCameraLocation, camera3DPoint(1:3)];
                globalStatic(end+1,1) = 0;
                globalObjectId(end+1,1) = frameFeatures.objectId(i);
            end
        end
        % globalAssociation: each row contains a unique object id and its tracklets of points
        if isempty(globalAssociation)
            globalAssociation{1,1} = frameFeatures.objectId(i);
            globalAssociation{1,2} = size(globalLocation3D,2);
        else
            % new dynamic 3D point - start new tracklet
            if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) > 0.001)
                objectIds = [globalAssociation{:,1}];
                idx = find(objectIds == frameFeatures.objectId(i));
                % new tracklet on an existing object
                if ~isempty(idx)
                    l = size(globalAssociation(idx,:),2);
                    globalAssociation{idx,l+1} = size(globalLocation3D,2);
                % new tracklet on a new object
                else
                    l = size(globalAssociation,1);
                    globalAssociation{l+1,1} = frameFeatures.objectId(i);
                    globalAssociation{l+1,2} = size(globalLocation3D,2);
                end
            end
        end 
    end
end
frameFeatures.location3D = featuresLocation3D;

globalFeatures.location3D = globalLocation3D;
globalFeatures.weight = globalWeight;
globalFeatures.id = globalId;
globalFeatures.frame = globalFrame;
globalFeatures.cameraLocation = globalCameraLocation;
globalFeatures.static = globalStatic;
globalFeatures.objectId = globalObjectId;
globalFeatures.dynamicAssociation = globalAssociation;

end