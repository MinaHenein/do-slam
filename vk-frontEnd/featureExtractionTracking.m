function frames = featureExtractionTracking(imageRange,K,rgbI,depthI,maskI,...
    motFile,cameraPoses,nFeaturesPerFrame,nFeaturesPerObject,maxStaticFeaturesPerFrame)

% setup 
frames(1).features.location = [];
frames(1).features.objectId = [];

for i = 1:numel(imageRange)
    fprintf('frame #%d\n',i);
    
    % frame i
    frameNumber = imageRange(i);
    frames(i).number = imageRange(i);
    frameName = strcat(repmat('0',1,5-numel(num2str(frameNumber))),num2str(frameNumber),'.png');
    frames(i).cameraPose = cameraPoses(:,i);
    if i == 1
        % extract objects
        maskIm = imread(strcat(maskI, frameName));
        frameObjects = extractFrameObjects(maskIm,motFile,frames(1));
        frames(1).objects = frameObjects;
    end
    % extract features
    rgbIm = imread(strcat(rgbI,frameName));
    depthIm = imread(strcat(depthI,frameName));
    frameFeatures = extractFrameFeatures(K,rgbIm,depthIm,frames(i).objects,...
        frames(i),nFeaturesPerFrame-length(frames(i).features.location),nFeaturesPerObject);
    if isempty(frames(i).features.location)
        frames(i).features = frameFeatures;
    else
        if ~isempty(frameFeatures.location)
            frames(i).features.location = [frames(i).features.location; frameFeatures.location];
            frames(i).features.moving = [frames(i).features.moving; frameFeatures.moving];
            frames(i).features.objectId = [frames(i).features.objectId; frameFeatures.objectId];
            frames(i).features.location3D = [frames(i).features.location3D, frameFeatures.location3D];
            frames(i).features.originFrame = [frames(i).features.originFrame; frameFeatures.originFrame];
        end
    end
    frame = postProcessFrameFeatures(frames(i),maxStaticFeaturesPerFrame);
    frames(i) = frame;
    
    if i < numel(imageRange)
        % frame i+1
        nextFrameNumber = imageRange(i+1);
        frames(i+1).number = imageRange(i+1);
        nextFrameName = strcat(repmat('0',1,5-numel(num2str(nextFrameNumber))),num2str(nextFrameNumber),'.png');
        frames(i+1).cameraPose = cameraPoses(:,i+1);
        % extract objects
        nextMaskIm = imread(strcat(maskI, nextFrameName));
        nextFrameObjects = extractFrameObjects(nextMaskIm,motFile,frames(i+1));
        frames(i+1).objects = nextFrameObjects;
        % project last frame features
        nextFrameFeatures = projectFeaturesForward(frames(i),K,frames(i+1),size(rgbIm));
        frames(i+1).features = nextFrameFeatures;
    end
    
end





end