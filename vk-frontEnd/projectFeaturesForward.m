function [nextFrameFeatures,globalFeatures] = projectFeaturesForward(frame,flowI,K,nextFrame,...
    nextRGBI,nextDepthI,globalFeatures,settings)

featuresLocation = [];
featuresObjectId = [];
featuresMoving = [];
featuresLocation3D = [];
featuresOriginFrame = [];
featuresId = [];

globalLocation3D = globalFeatures.location3D;
globalWeight = globalFeatures.weight;
globalId = globalFeatures.id;
globalFrame = globalFeatures.frame;
globalCameraLocation = globalFeatures.cameraLocation;
globalStatic = globalFeatures.static;
globalObjectId = globalFeatures.objectId;
globalAssociation = globalFeatures.dynamicAssociation;

for i=1:size(frame.features.location,1)
    world3DPoint = frame.features.location3D(:,i);
    % compute distance to all 3D points
    distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
        (globalLocation3D(1,:).'-world3DPoint(1,1)).^2,...
        (globalLocation3D(2,:).'-world3DPoint(2,1)).^2),...
        (globalLocation3D(3,:).'-world3DPoint(3,1)).^2))';
    % find min distant point
    [~,index] = find(distances == min(distances));
    switch settings.featureMatchingMethod
        case 'GT'
            nextCameraPoseMatrix = poseToTransformationMatrix(nextFrame.cameraPose);
            % static point
            if ~frame.features.moving(i)
                nextCamera3DPoint = nextCameraPoseMatrix\[world3DPoint;1];
                nextCamera3DPoint = nextCamera3DPoint(1:3,1);
                % camera --> image
                nextImagePoint = K * nextCamera3DPoint;
                nextImagePoint = nextImagePoint/nextImagePoint(3);
                if isPointWithinImageSize(nextImagePoint,size(nextRGBI))
                    corners = detectFASTFeatures(rgb2gray(nextRGBI),'ROI',[max(1,nextImagePoint(1)-30),...
                        max(1,nextImagePoint(2)-30),...
                        min(60,size(nextRGBI,2)-nextImagePoint(1)),min(60,size(nextRGBI,1)-nextImagePoint(2))]);
                    if corners.Count ~=0
                        distances = sqrt(bsxfun(@plus,...
                            (corners.Location(:,1).'-nextImagePoint(1,1)).^2,...
                            (corners.Location(:,2).'-nextImagePoint(2,1)).^2))';
%                         if min(distances) > 3
%                             continue;
%                         end
                        cornerIndex = find(distances == min(distances));
                        nextImagePoint = corners.Location(cornerIndex,:);
                        
                        featuresLocation = [featuresLocation; nextImagePoint(1:2)];
                        featuresObjectId = [featuresObjectId; -1];
                        featuresMoving = [featuresMoving;0];
                        featuresLocation3D = [featuresLocation3D, frame.features.location3D(:,i)];
                        featuresOriginFrame = [featuresOriginFrame; frame.features.originFrame(i)];
                        featuresId = [featuresId; frame.features.id(i)];
                        % if closest point is within distance threshold in 3D
                        if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) <= settings.distanceThreshold)
                            globalWeight(index,1) = globalWeight(index,1) + 1;
                            if(globalStatic(index,1) ~= 1 || globalObjectId(index,1) ~= -1)
                                continue;
                            end
                            %assert(globalStatic(index,1) == 1);
                            %assert(globalObjectId(index,1) == -1);
                        else
                            disp('error!');
                            disp('static point detected in previous frame should already be in global features structure');
                        end
                    end
                end
            else
                % dynamic point
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
                    if isPointWithinImageSize(nextImagePoint,size(nextRGBI))
                        corners = detectFASTFeatures(rgb2gray(nextRGBI),'ROI',[max(1,nextImagePoint(1)-30),...
                            max(1,nextImagePoint(2)-30),...
                            min(60,size(nextRGBI,2)-nextImagePoint(1)),min(60,size(nextRGBI,1)-nextImagePoint(2))]);
                        if corners.Count ~=0
                            distances = sqrt(bsxfun(@plus,...
                                (corners.Location(:,1).'-nextImagePoint(1,1)).^2,...
                                (corners.Location(:,2).'-nextImagePoint(2,1)).^2))';
%                             if min(distances) > 3
%                                 continue;
%                             end
                            cornerIndex = find(distances == min(distances));
                            nextImagePoint = corners.Location(cornerIndex,:);
                            featuresLocation = [featuresLocation; nextImagePoint(1:2)];
                            featuresObjectId = [featuresObjectId; objectId];
                            featuresMoving = [featuresMoving;nextFrame.objects(nextIndx).moving];
                            featuresLocation3D = [featuresLocation3D, movedWorld3DPoint(1:3)];
                            featuresOriginFrame = [featuresOriginFrame; nextFrame.number];
                            featuresId = [featuresId; size(globalLocation3D,2)+1];
                            globalLocation3D = [globalLocation3D, movedWorld3DPoint(1:3)];
                            globalWeight(end+1,1) = 1;
                            globalId(end+1,1) = featuresId(end);
                            globalFrame(end+1,1) = featuresOriginFrame(end);
                            globalCameraLocation = [globalCameraLocation, nextCamera3DPoint];
                            globalStatic(end+1,1) = 0;
                            globalObjectId(end+1,1) = featuresObjectId(end);
                            % global associations
                            objectIds = [globalAssociation{:,1}];
                            idx = find(objectIds == featuresObjectId(end));
                            if ~isempty(idx)
                                if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) <= settings.distanceThreshold+0.0001)
                                    assert(globalStatic(index,1) == 0);
                                    assert(globalObjectId(index,1) == featuresObjectId(end));
                                else
                                    disp('error!')
                                    disp('dynamic point detected in previous frame should already be in global features structure')
                                end
                                % find index of world3DPoint in globalLocation3D
                                originFeatureId = globalId(index,1);
                                for j = 2:size(globalAssociation(idx,:),2)
                                    if ismember(originFeatureId, [globalAssociation{idx,j}])
                                        globalAssociation{idx,j} = [globalAssociation{idx,j}, size(globalLocation3D,2)];
                                        break;
                                    end
                                end
                            else
                                disp('error!')
                                disp('dynamic point detected in previous frame, object id should already be in global associations structure')
                            end
                        end
                    end
                end
            end
        case 'PWC-Net'
            imagePixel = frame.features.location(i,:);
            pixelRow = frame.features.location(i,2);
            pixelCol = frame.features.location(i,1);
            pixelFlow = flowI(pixelRow,pixelCol,:);
            nextImagePoint = [imagePixel + [pixelFlow(:,:,1) pixelFlow(:,:,2)]]';
            if isPointWithinImageSize(nextImagePoint,size(nextRGBI))
                corners = detectFASTFeatures(rgb2gray(nextRGBI),'ROI',[max(1,nextImagePoint(1)-30),...
                    max(1,nextImagePoint(2)-30),...
                    min(60,size(nextRGBI,2)-nextImagePoint(1)),min(60,size(nextRGBI,1)-nextImagePoint(2))]);
                if corners.Count ~=0
                    distances = sqrt(bsxfun(@plus,...
                        (corners.Location(:,1).'-nextImagePoint(1,1)).^2,...
                        (corners.Location(:,2).'-nextImagePoint(2,1)).^2))';
                    cornerIndex = find(distances == min(distances));
%                     if min(distances) > 3
%                         continue;
%                     end
                    nextImagePoint = corners.Location(cornerIndex,:);
                    % static point
                    if ~frame.features.moving(i)
                        featuresLocation = [featuresLocation; nextImagePoint(1:2)];
                        featuresObjectId = [featuresObjectId; -1];
                        featuresMoving = [featuresMoving;0];
                        featuresLocation3D = [featuresLocation3D, frame.features.location3D(:,i)];
                        featuresOriginFrame = [featuresOriginFrame; frame.features.originFrame(i)];
                        featuresId = [featuresId; frame.features.id(i)];
                        % if closest point is within 1 mm in 3D
                        if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) < settings.distanceThreshold)
                            globalWeight(index,1) = globalWeight(index,1) + 1;
                            if(globalStatic(index,1) ~= 1 || globalObjectId(index,1) ~= -1)
                                continue;
                            end
                            %assert(globalStatic(index,1) == 1);
                            %assert(globalObjectId(index,1) == -1);
                        else
                            disp('error!');
                            disp('static point detected in previous frame should already be in global features structure');
                        end
                        
                    else
                        % dynamic point
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
                            nextIndx = find(nextFrameObjectIds == objectId);
                            featuresLocation = [featuresLocation; nextImagePoint(1:2)];
                            featuresObjectId = [featuresObjectId; objectId];
                            featuresMoving = [featuresMoving;nextFrame.objects(nextIndx).moving];
                            
                            % image--> camera
                            nextPixelRow = nextImagePoint(1,2);
                            nextPixelCol = nextImagePoint(1,1);
                            nextPixelDisparity = double(nextDepthI(nextPixelRow,nextPixelCol))/256;
                            nextPixelDepth = K(1,1)*0.537/nextPixelDisparity;
                            nextCamera3DPoint = K\[nextPixelCol;nextPixelRow;1];
                            nextCamera3DPoint = nextCamera3DPoint * nextPixelDepth;
                            % camera --> world
                            nextCameraPoseMatrix = poseToTransformationMatrix(nextFrame.cameraPose);
                            movedWorld3DPoint = nextCameraPoseMatrix * [nextCamera3DPoint;1];
                            featuresLocation3D = [featuresLocation3D, movedWorld3DPoint(1:3)];
                            featuresOriginFrame = [featuresOriginFrame; nextFrame.number];
                            featuresId = [featuresId; size(globalLocation3D,2)+1];
                            globalLocation3D = [globalLocation3D, movedWorld3DPoint(1:3)];
                            globalWeight(end+1,1) = 1;
                            globalId(end+1,1) = featuresId(end);
                            globalFrame(end+1,1) = featuresOriginFrame(end);
                            globalCameraLocation = [globalCameraLocation, nextCamera3DPoint];
                            globalStatic(end+1,1) = 0;
                            globalObjectId(end+1,1) = featuresObjectId(end);
                            % global associations
                            objectIds = [globalAssociation{:,1}];
                            idx = find(objectIds == featuresObjectId(end));
                            if ~isempty(idx)
                                if(norm(world3DPoint(1:3,1) - globalLocation3D(:,index)) < settings.distanceThreshold)
                                    for m = 1:length(index)
                                        % if point has changed object or is no longer moving, skip
                                        if globalStatic(index(m),1) ~= 0 || globalObjectId(index(m),1) ~= featuresObjectId(end)
                                            continue;
                                        end
                                    end
                                else
                                    disp('error!')
                                    disp('dynamic point detected in previous frame should already be in global features structure')
                                end
                                % find index of world3DPoint in globalLocation3D
                                originFeatureId = globalId(index,1);
                                for j = 2:size(globalAssociation(idx,:),2)
                                    if ismember(originFeatureId, [globalAssociation{idx,j}])
                                        globalAssociation{idx,j} = [globalAssociation{idx,j}, size(globalLocation3D,2)];
                                        break;
                                    end
                                end
                            else
                                disp('error!')
                                disp('dynamic point detected in previous frame, object id should already be in global associations structure')
                            end
                        end
                    end
                end
            end
    end
end
nextFrameFeatures.location = featuresLocation;
nextFrameFeatures.objectId = featuresObjectId;
nextFrameFeatures.moving = featuresMoving;
nextFrameFeatures.location3D = featuresLocation3D;
nextFrameFeatures.originFrame = featuresOriginFrame;
nextFrameFeatures.id = featuresId;

globalFeatures.location3D = globalLocation3D;
globalFeatures.weight = globalWeight;
globalFeatures.id = globalId;
globalFeatures.frame = globalFrame;
globalFeatures.cameraLocation = globalCameraLocation;
globalFeatures.static = globalStatic;
globalFeatures.objectId = globalObjectId;
globalFeatures.dynamicAssociation = globalAssociation;

end