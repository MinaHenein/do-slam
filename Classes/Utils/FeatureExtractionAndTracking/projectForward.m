function [framePoints,uniqueValid3DPointsWeights,featureCount,...
features,uniqueValid3DPointsCameras] = projectForward(start,n,framePoints,...
firstFrame,increment,desiredPointsPerFrame,depthFilePath,K_Cam,PWorld,...
uniqueValid3DPoints,uniqueValid3DPointsWeights,featureCount,features,...
uniqueValid3DPointsCameras,syncedData,gtFilePath)

rgbFileName = config.rgbImageName;
rgbFileNameFormat = getFileNameFormat(rgbFileName);
depthFileName = config.depthImageName;
depthFileNameFormat = getFileNameFormat(depthFileName);
depthFileNameExtension = config.rgbImageFileExtension;

for i=1:1:n-1
    if(framePoints(floor((start+i*increment-(firstFrame-increment))/increment),1) ...
            < desiredPointsPerFrame)
        % get camera pose from gt file
        fid = fopen(gtFilePath);
        if ~isempty(syncedData)
            gtInSyncedData = length(depthFileNameFormat)+length(rgbFileNameFormat)+3;
            lineScan = textscan(fid,'%s',1,'delimiter','\n','headerlines',...
                str2double(syncedData(i,gtInSyncedData:end))-1);
        else
            lineScan = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines', i-1);
        end
        nextCameraIDPose = str2num(cell2mat(strsplit(cell2mat(lineScan{1,1}),'')));
        fclose(fid);
        nextCameraPose = nextCameraIDPose(2:end);
        nextCameraTranslation = nextCameraPose(1:3)';
        nextCameraRotation = quaternion2Axis([nextCameraPose(4);nextCameraPose(5);...
            nextCameraPose(6);nextCameraPose(7)]);
        nextCameraPose = [nextCameraTranslation;nextCameraRotation];
        % get measurement to next camera
        pointInNextCamFrame = AbsoluteToRelativePosition(nextCameraPose,PWorld(1:3,1));
        % get pixel coordinates in image to double check matches
        imagePoint = K_Cam * pointInNextCamFrame;
        if imagePoint(3,1)~=0
        imagePoint = imagePoint/imagePoint(3,1);
        end
        if(isPointWithinImageSize(imagePoint,[640,480]))
            % get depth file
            if ~isempty(syncedData)
                depthInSyncedData = 1:length(depthFileNameFormat);
                depthFile= strcat(syncedData(i,depthInSyncedData),depthFileNameExtension);
                nextImgDepth = reshape(imread(strcat(depthFilePath,'/',depthFile)),640, 480)';
            else
                nextImgDepth = reshape(load(strcat(depthFilePath,'/',depthFileName,num2str(i),...
                    depthFileNameExtension)),640,480)';
            end
            nextImgDepth = double(nextImgDepth);
            pixelLocation = imagePoint;
            PCamera = K_Cam \ pixelLocation;
            zCamera = nextImgDepth(round(pixelLocation(2,1)), ...
                round(pixelLocation(1,1)))/1000;
            nextCameraToWorld = [rot(nextCameraRotation), ...
                nextCameraTranslation; 0 0 0 1];
            PCamera = PCamera*zCamera;
            PWorld2 = nextCameraToWorld * [PCamera; 1];
            if(norm(PWorld-PWorld2)<0.1)
                % increase weight of 3D Point
                distances = sqrt(bsxfun(@plus,bsxfun(@plus,...
                    (uniqueValid3DPoints(:,1).'-PWorld(1,1)).^2,...
                    (uniqueValid3DPoints(:,2).'-PWorld(2,1)).^2),...
                    (uniqueValid3DPoints(:,3).'-PWorld(3,1)).^2))';
                [ptID,~] = find(distances==min(distances));
                uniqueValid3DPointsWeights(ptID)=uniqueValid3DPointsWeights(ptID)+1;
                % increase framePoints
                framePoints(floor((start+i*increment-(firstFrame-increment))/increment),1)=...
                    framePoints(floor((start+i*increment-(firstFrame-increment))/increment),1)+1;
                % add frame features
                featureCount = featureCount+1;
                features(featureCount,:) = [start+i*increment,...
                    imagePoint(1,1),imagePoint(2,1)];
                % add camera id
                uniqueValid3DPointsCameras{ptID,end+1} = start+i*increment;
            end
        end
    end
end

end