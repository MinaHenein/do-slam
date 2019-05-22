function objectPosesCell = saveGTObjectPoses(imageRange, objectDetectionFile, cameraExtrinsicsFile)

fid = fopen(objectDetectionFile);
Data = textscan(fid,'%s','Delimiter','\n');
cellStr = Data{1};
fclose(fid);

% determine number od objects in sequence and their ids
objectIds =  [];
for i = imageRange
    for j = 2:length(cellStr)
        splitLine = strsplit(cellStr{j,1},' ');
        if str2double(splitLine{1,1}) == i && strcmp(splitLine{1,3},'Car')
            % object id
            id = str2double(splitLine{1,2});
            if isempty(objectIds)
                objectIds =  [objectIds, id];
            elseif ~ismember(id, objectIds)
                objectIds =  [objectIds, id];
            end
        end
    end
end

objectPosesCell = {};
for i = imageRange 
    % get camera extrinsics matrix
    fid = fopen(cameraExtrinsicsFile);
    lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+1);
    fclose(fid);
    lineArray = str2num(cell2mat(lineCell{1,1}));
    assert(lineArray(1)==i);
    cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
    % get all objects in frame i
    for j = 2:length(cellStr)
        splitLine = strsplit(cellStr{j,1},' ');
        if str2double(splitLine{1,1}) == i && strcmp(splitLine{1,3},'Car')
            % object id
            id = str2double(splitLine{1,2});
            % object pose in camera frame
            x3d = str2double(splitLine{1,14});
            y3d = str2double(splitLine{1,15});
            z3d = str2double(splitLine{1,16});
            yaw = str2double(splitLine{1,17});
            pitch = str2double(splitLine{1,18});
            roll = str2double(splitLine{1,19});
            Ry = eul2Rot([0, yaw + pi/2, 0]);
            Rx = eul2Rot([0, 0, pitch]);
            Rz = eul2Rot([roll, 0, 0]);
            R =  Ry * Rx * Rz ;
            objectTranslationCameraFrame = [x3d;y3d;z3d];
            objectRotationCameraFrame = R;
            objectPoseCameraFrame = [objectRotationCameraFrame, objectTranslationCameraFrame; 0 0 0 1];
            objectPoseWorldFrame = cameraPoseMatrix * objectPoseCameraFrame;
            if size(objectPosesCell,2) < find(objectIds == id)
                objectPosesCell{find(objectIds == id)} = [transformationMatrixToPose(objectPoseWorldFrame)];
            else
                objectPosesCell{find(objectIds == id)} = [objectPosesCell{find(objectIds == id)}, ...
                transformationMatrixToPose(objectPoseWorldFrame)];
            end
            
        end
    end
end

end