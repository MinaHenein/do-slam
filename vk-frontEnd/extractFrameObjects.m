function frameObjects = extractFrameObjects(maskI, motFile, frame, settings)

% read mot file
fileID = fopen(motFile,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);

% number of objects in frame
nObjects = 0;
frameObjects = [];
for i=1:numel(CStr)-1
    lineCell = strsplit(CStr{i+1,1},' '); % +1 to skip 1st line
    if (str2double(lineCell{1,1}) == frame.number)
        label = lineCell{3};
        if strcmp(settings.dataset,'kitti')
            id = str2double(lineCell{2});
            maskColour = 1000 + id;
            %maskColour = [];
        elseif strcmp(settings.dataset,'vkitti')
            maskColour = [];
        end
        if strcmp(label,'Car')
        %if (strcmp(label,'Van') || strcmp(label,'Cyclist')) && str2double(lineCell{1,2})<3
            % object bounding box in pixels most left,top,right,bottom
            l = str2double(lineCell{1,7});
            t = str2double(lineCell{1,8});
            r = str2double(lineCell{1,9});
            b = str2double(lineCell{1,10});
            % object binary mask
            objectMask = getObjectMask(l,t,r,b,maskI,maskColour,settings);
            [nRows, nCols, ~] = size(maskI);
            % object occupies more than 6% of image
            if 100*sum(sum(objectMask))/(nRows*nCols) > 1
                nObjects = nObjects+1;
                % assign object bounding box
                frameObjects(nObjects).boundingBox = [l,t,r,b];
                % assign object binary mask
                frameObjects(nObjects).mask = objectMask;
                % object label {'Car', 'Van', ...}
                frameObjects(nObjects).label = label;
                % object unique tracking id
                frameObjects(nObjects).id = str2double(lineCell{1,2});
                % moving or static
                % no motion segmentation provided for kitti - consider all
                % moveable objects
                if strcmp(settings.dataset,'kitti')
                   frameObjects(nObjects).moving = 1; 
                elseif strcmp(settings.dataset,'vkitti')
                   frameObjects(nObjects).moving = str2double(lineCell{1,23});
                end
                % object pose in camera frame
                x3d = str2double(lineCell{1,14});
                y3d = str2double(lineCell{1,15});
                z3d = str2double(lineCell{1,16});
                yaw = str2double(lineCell{1,17});
                if strcmp(settings.dataset,'kitti')
                    pitch = 0;
                    roll = 0;
                elseif strcmp(settings.dataset,'vkitti')
                    pitch = str2double(lineCell{1,18});
                    roll = str2double(lineCell{1,19});
                end
                Ry = eul2Rot([0, yaw + pi/2, 0]);
                Rx = eul2Rot([0, 0, pitch]);
                Rz = eul2Rot([roll, 0, 0]);
                R =  Ry * Rx * Rz ;
                objectTranslationCameraFrame = [x3d;y3d;z3d];
                objectRotationCameraFrame = R;
                objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
                frameObjects(nObjects).poseCameraFrame = transformationMatrixToPose(objectPoseCameraFrame);
                % object pose in world frame
                cameraPoseMatrix = poseToTransformationMatrix(frame.cameraPose);
                objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
                frameObjects(nObjects).poseWorldFrame = transformationMatrixToPose(objectPoseWorldFrame);
            end
        end
    elseif (str2double(lineCell{1,1}) > frame.number)
        break
    end
end

end