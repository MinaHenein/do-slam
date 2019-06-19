function nextFrameFeatures = projectFeaturesForward(frame,K,nextFrame,rgbISize)

featuresLocation = [];
featuresObjectId = [];
featuresMoving = [];
featuresLocation3D = [];
featuresOriginFrame = [];
for i=1:length(frame.features.location)
    world3DPoint = frame.features.location3D(:,i);
    nextCameraPoseMatrix = poseToTransformationMatrix(nextFrame.cameraPose);
    % static background point
    if ~frame.features.moving(i)
        nextCamera3DPoint = nextCameraPoseMatrix\[world3DPoint;1];
        nextCamera3DPoint = nextCamera3DPoint(1:3,1);
        % camera --> image
        nextImagePoint = K * nextCamera3DPoint;
        nextImagePoint = nextImagePoint/nextImagePoint(3);
        if isPointWithinImageSize(nextImagePoint,rgbISize)
            featuresLocation = [featuresLocation; nextImagePoint(1:2)'];
            featuresObjectId = [featuresObjectId; -1];
            featuresMoving = [featuresMoving;0];
            featuresLocation3D = [featuresLocation3D, frame.features.location3D(:,i)];
            featuresOriginFrame = [featuresOriginFrame; frame.features.originFrame(i)];
        end
    else
        % possibly dynamic point (moveable object point)
        objectId = frame.features.objectId(i);
        if ~isempty(frame.objects)
            currentFrameObjectIds = [frame.objects.id];
        else
            currentFrameObjectIds = [];
        end
        
        if ~isempty(nextFrame.objects)
            nextFrameObjectIds = [nextFrame.objects.id];
        else
            nextFrameObjectIds = [];
        end
        if ismember(objectId,currentFrameObjectIds) && ismember(objectId,nextFrameObjectIds)
            indx = find(currentFrameObjectIds == objectId);
            objectPoseWorldFrame = poseToTransformationMatrix(frame.objects(indx).poseWorldFrame);
            nextIndx = find(nextFrameObjectIds == objectId);
            nextObjectPoseWorldFrame = poseToTransformationMatrix(nextFrame.objects(nextIndx).poseWorldFrame);
            objectMotionWorldFrame = nextObjectPoseWorldFrame/objectPoseWorldFrame;
            movedWorld3DPoint = objectMotionWorldFrame * [world3DPoint;1];
            % project onto next frame
            nextCamera3DPoint = nextCameraPoseMatrix \ movedWorld3DPoint;
            nextCamera3DPoint = nextCamera3DPoint(1:3,1);
            % camera --> image
            nextImagePoint = K * nextCamera3DPoint;
            nextImagePoint = nextImagePoint/nextImagePoint(3);
            if isPointWithinImageSize(nextImagePoint,rgbISize)
                featuresLocation = [featuresLocation; nextImagePoint(1:2)'];
                featuresObjectId = [featuresObjectId; objectId];
                featuresMoving = [featuresMoving;nextFrame.objects(nextIndx).moving];
                featuresLocation3D = [featuresLocation3D, movedWorld3DPoint(1:3)];
                featuresOriginFrame = [featuresOriginFrame; frame.features.originFrame(i)];
            end
        end
    end
end
nextFrameFeatures.location = featuresLocation;
nextFrameFeatures.moving = featuresMoving;
nextFrameFeatures.objectId = featuresObjectId;
nextFrameFeatures.originFrame = featuresOriginFrame;
nextFrameFeatures.location3D = featuresLocation3D;


end