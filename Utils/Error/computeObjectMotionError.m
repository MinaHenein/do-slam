function [translationError, rotationError, speedError] = ...
    computeObjectMotionError(resultFileName,gtFileName,objectPosesMatrix,...
    constantMotionAssumption,objectIndexCell,startFrame, endFrame)

if constantMotionAssumption
    averageGTObjectMotions = vKitti_objectMotionAveraged(objectPosesMatrix);
else
    [gtObjectMotions,gtObjectFrames] = vKitti_objectMotion(objectPosesMatrix);
end

gtFilePath = strcat(pwd,'/Data/GraphFiles/',gtFileName);
objectPoints = vKITTI_pointObservability(gtFilePath);
fid = fopen(gtFilePath, 'r');
Data = textscan(fid,'%s','Delimiter','\n');
gtCStr = Data{1};
fclose(fid);
posesIndex = find(contains(gtCStr, 'VERTEX_POSE_R3_SO3')==1);
dataAssociationIndex = find(contains(gtCStr,'2PointsDataAssociation')==1);
            
% if constantMotionAssumption
%     searchedStr = '2PointsDataAssociation';
% else
%     searchedStr = '2OBJECTS_DataAssociation';
% end
% IndexC = strfind(gtCStr, searchedStr);
% Index = find(~cellfun('isempty', IndexC));
% if constantMotionAssumption
%     objectIdCell = {};
%     for i=1:length(Index)
%         splitLine = strsplit(gtCStr{Index(i),1},' ');
%         index = str2double(splitLine{1,4});
%         objectIdCell{i,1} = index;
%     end
% else
%     objectIndices = zeros(length(Index),2);
%     for i=1:length(Index)
%         splitLine = strsplit(gtCStr{Index(i),1},' ');
%         index1 = str2double(splitLine{1,2});
%         index2 = str2double(splitLine{1,3});
%         objectIndices(i,:) = [index1, index2];
%     end
%     objectIdCell = {};
%     for i=1:size(objectIndices,1)
%         if i==1
%             objectIdCell{1} = [objectIndices(1,1), objectIndices(1,2)];
%         else
%             objectIndex = objectIndices(i,1);
%             if ismember(objectIndex,[objectIndices(1:i-1,:)])
%                 for j=1:length(objectIdCell)
%                     if ~isempty(find([objectIdCell{j,:}] == objectIndex))
%                         break
%                     end
%                 end
%                 objectIdCell{j} = [objectIdCell{j}, objectIndices(i,2)];
%             else
%                 objectIdCell{end+1,1} = [objectIndices(i,1), objectIndices(i,2)];
%             end
%         end
%     end
% end

objectIndices = [];
for i=1:length(dataAssociationIndex)
    line = gtCStr{dataAssociationIndex(i),1};
    splitLine = strsplit(line,' ');
    objectId = str2double(splitLine(4));
    objectIndices = [objectIndices,objectId];
end
objectIndices = unique(objectIndices);
 
for i=1:length(objectIndexCell)
    objectIds = objectIndexCell(i).entry;
    toBeDeleted = [];
    for j=1:length(objectIds)
        if ~ismember(objectIds(j),objectIndices)
           toBeDeleted = [toBeDeleted, j];
        end
    end
    objectIds(toBeDeleted) = [];
    objectIndexCell(i).entry = objectIds;
end

resultFilepath = strcat(pwd,'/Data/GraphFiles/',resultFileName);
resultFileID = fopen(resultFilepath,'r');
resultData = textscan(resultFileID, '%s', 'delimiter', '\n', 'whitespace', '');
resultCStr = resultData{1};
fclose(resultFileID);
%% object motions
if constantMotionAssumption
    averageSE3ObjectMotions = zeros(6,length(objectIndexCell));
    for i=1:length(objectIndexCell)
        currentObjectIndex = objectIndexCell(i).entry;
        IndexC = strfind(resultCStr, strcat({'VERTEX_SE3Motion'},{' '},...
            {num2str(currentObjectIndex)},{' '}));
        lineIndex = find(~cellfun('isempty', IndexC));
        splitLine = strsplit(resultCStr{lineIndex,1},' ');
        value = str2double(splitLine(3:end));
        averageSE3ObjectMotions(:,i) = value';
    end
else
    objectMotions = {};
    objectMotionFrames = {};
    for i=1:length(objectIndexCell)
        currentObjectMotionIndices = [objectIndexCell(i).entry];
        currentObjectMotions =  zeros(6,length(currentObjectMotionIndices));
        currentObjectMotionFrames = zeros(1,length(currentObjectMotionIndices));
        for j = 1:length(currentObjectMotionIndices)
            IndexC = strfind(resultCStr, strcat({'VERTEX_SE3Motion'},{' '},...
                {num2str(currentObjectMotionIndices(j))},{' '}));
            lineIndex = find(~cellfun('isempty', IndexC));
            for k=1:length(dataAssociationIndex)
                split =  strsplit(gtCStr{dataAssociationIndex(k),1},' ');
                if str2double(split(end))==currentObjectMotionIndices(j)
                    break
                end
            end
            frame = length(find(posesIndex(posesIndex(:,1)<dataAssociationIndex(k))));
            currentObjectMotionFrames(1,j) = frame;
            if ~isempty(lineIndex)
                splitLine = strsplit(resultCStr{lineIndex,1},' ');
                value = str2double(splitLine(3:end));
                currentObjectMotions(:,j) = value';
            end
        end
        objectMotions{i} = currentObjectMotions;
        objectMotionFrames{i} = currentObjectMotionFrames;
    end    
end

%% object motions errors
objectMotionGTTranslation = zeros(1,length(objectIndexCell));
if constantMotionAssumption
    objectMotionError = zeros(6,length(objectIndexCell));
    for i=1:length(objectIndexCell)
        gtMotion = averageGTObjectMotions(:,i);
        currentMotion = averageSE3ObjectMotions(:,i);
        error = AbsoluteToRelativePoseR3xso3(currentMotion,gtMotion);
        objectMotionError(:,i) = error;
        averageObjectMotionTranslationError(1,i) = norm(error(1:3))/norm(gtMotion(1:3));
        averageObjectMotionRotationError(1,i) = norm(error(4:6))/norm(gtMotion(1:3));
        % for speed calculation
        objectGTMotion = averageGTObjectMotions(:,i);
        objectMotionGTTranslation(1,i) = norm(objectGTMotion(1:3));
    end
else
    objectMotionError = {};
    %% wrong!!
    for i=1:min(length(objectIndexCell),length(gtObjectMotions))
        currentObjectMotions = objectMotions{i};
        currentObjectMotions = currentObjectMotions(:,2:end);
        %currentObjectMotionFrames = objectMotionFrames{i};
        %currentObjectMotionFrames = currentObjectMotionFrames(2:end); 
        
        currentGTObjectMotions = gtObjectMotions{i};
        %currentGTObjectFrames = gtObjectFrames{i}; 
        %currentGTObjectMotionFrames = currentGTObjectFrames(2:end);
        
        currentObjectMotionError =  zeros(6,size(currentObjectMotions,2));
        currentObjectMotionTranslationError =  zeros(1,size(currentObjectMotions,2));
        currentObjectMotionRotationError =  zeros(1,size(currentObjectMotions,2));
        
        for j = 1:size(currentObjectMotions,2)
            %get gt object motion at frame number
%             indx = find(currentGTObjectMotionFrames == startFrame+currentObjectMotionFrames(j)-1);
%             if isempty(indx)
%                 continue
%             end
%             currentGTObjectMotion = currentGTObjectMotions(:,indx);
            currentGTObjectMotion = currentGTObjectMotions(:,j);
            currentObjectMotion = currentObjectMotions(:,j);
            error = AbsoluteToRelativePoseR3xso3(currentObjectMotion,currentGTObjectMotion);
            currentObjectMotionError(:,j) = error;
            currentObjectMotionTranslationError(1,j) = norm(error(1:3))/norm(currentGTObjectMotion(1:3));
            currentObjectMotionRotationError(1,j) = norm(error(4:6))/norm(currentGTObjectMotion(1:3));
        end
        objectMotionError{i} = currentObjectMotionError;
        averageObjectMotionTranslationError(1,i) = mean(currentObjectMotionTranslationError);
        averageObjectMotionRotationError(1,i) = mean(currentObjectMotionRotationError);
        objectMotionGTTranslation(1,i) = mean(norm(currentGTObjectMotions(1:3)));
    end
end

% if ~constantMotionAssumption
%     for i=1:length(objectMotionError)
%         objectMotionErrors = objectMotionError{i};
%         objectMotionErrors(:,~any(objectMotionErrors,1))=[];
%         objectMotionError{i} = objectMotionErrors;
%     end
% end
% %% per object average of object motions error
% if ~constantMotionAssumption
%     for i=1:length(objectIndexCell)
%         currentObjectMotionErros = objectMotionError{i};
%         objectTranslationError = zeros(1,size(currentObjectMotionErros,2));
%         objectRotationError = zeros(1,size(currentObjectMotionErros,2));
%         for j = 1:size(currentObjectMotionErros,2)
%             error = currentObjectMotionErros(:,j);
%             objectTranslationError(1,j) = norm(error(1:3));
%             objectRotationError(1,j) = norm(error(4:6));
%         end
%         averageObjectMotionTranslationError(1,i) = mean(objectTranslationError);
%         averageObjectMotionRotationError(1,i) = mean(objectRotationError);
%     end
% end

%% average of (per object average of object motions error)
% translationError = mean(averageObjectMotionTranslationError)/mean(objectMotionGTTranslation);
% rotationError = mean(averageObjectMotionRotationError)/mean(objectMotionGTTranslation);

translationError = mean(averageObjectMotionTranslationError);
rotationError = mean(averageObjectMotionRotationError);
%% object speeds errors
if constantMotionAssumption
    for i=1:length(objectIndexCell)
        %averageObjectSpeedErrors = zeros(1,length(objectIndexCell));
        currentObjectIndex = objectIndexCell(i).entry;
        IndexC = strfind(resultCStr, strcat({'VERTEX_SE3Motion'},{' '},...
            {num2str(currentObjectIndex)},{' '}));
        lineIndex = find(~cellfun('isempty', IndexC));
        splitLine = strsplit(resultCStr{lineIndex,1},' ');
        motionValue = str2double(splitLine(3:end));
        gtMotionValue = averageGTObjectMotions(:,i);
        objectSpeedErrors = [];
        for j = 1:endFrame-(startFrame-1)
            if size(objectPoints,2) >= j
                objectFramePoints = objectPoints{i,j};
                if ~isempty(objectFramePoints)
                    pointFramePositions = zeros(3,length(objectFramePoints));
                    pointFrameGTPositions = zeros(3,length(objectFramePoints));
                    for k=1:length(objectFramePoints)
                        IndexC = strfind(resultCStr, strcat({'VERTEX_POINT_3D'},{' '},...
                            {num2str(objectFramePoints(k))},{' '}));
                        lineIndex = find(~cellfun('isempty', IndexC));
                        splitLine = strsplit(resultCStr{lineIndex,1},' ');
                        pointFramePositions(:,k) = str2double(splitLine(3:end))';
                        
                        IndexC = strfind(gtCStr, strcat({'VERTEX_POINT_3D'},{' '},...
                            {num2str(objectFramePoints(k))},{' '}));
                        lineIndex = find(~cellfun('isempty', IndexC));
                        splitLine = strsplit(gtCStr{lineIndex,1},' ');
                        pointFrameGTPositions(:,k) = str2double(splitLine(3:end))';
                    end
                    centroid = mean(pointFramePositions,2);
                    gtCentroid = mean(pointFrameGTPositions,2);
                    speed = norm(motionValue(1:3)' - (eye(3)-rot(motionValue(4:6)'))*centroid);
                    gtSpeed = norm(gtMotionValue(1:3) - (eye(3)-rot(gtMotionValue(4:6)))*gtCentroid);
                    objectSpeedErrors = [objectSpeedErrors, abs(gtSpeed-speed)/gtSpeed];
                end
            end
        end
        averageObjectSpeedErrors(1,i) = mean(objectSpeedErrors);
    end
else
    objectSpeedErrors = {};
    %% wrong!!
    for i=1:min(length(objectIndexCell),length(gtObjectMotions))    
        currentObjectIds = objectIndexCell(i).entry;
        currentObjectMotions = objectMotions{i};
        currentObjectMotions = currentObjectMotions(:,2:end);
        currentObjectMotionFrames = objectMotionFrames{i};
        currentGTObjectMotions = gtObjectMotions{i};
        currentGTObjectFrames = gtObjectFrames{i}; 
        currentGTObjectMotionFrames = currentGTObjectFrames(2:end)-1;
        for j = 1:size(currentObjectMotions,2)
            %averageObjectSpeedErrors = zeros(1,size(currentObjectMotions,2));
            motionValue = currentObjectMotions(:,j);
            %get gt object motion at frame number
%             indx = find(currentGTObjectMotionFrames == startFrame+currentObjectMotionFrames(j)-1);
%             if isempty(indx)
%                 continue
%             end
%             gtMotionValue = currentGTObjectMotions(:,indx);
            gtMotionValue = currentGTObjectMotions(:,j);
            currentObjectSpeedErrors = [];
            for k = 1:endFrame-(startFrame-1)
                if size(objectPoints,2) >= k
                    objectIndx = find(objectIndices == currentObjectIds(j));
                    objectFramePoints = objectPoints{objectIndx,k};
                    if ~isempty(objectFramePoints)
                        pointFramePositions = zeros(3,length(objectFramePoints));
                        pointFrameGTPositions = zeros(3,length(objectFramePoints));
                        for m = 1:length(objectFramePoints)
                            IndexC = strfind(resultCStr, strcat({'VERTEX_POINT_3D'},{' '},...
                                {num2str(objectFramePoints(m))},{' '}));
                            lineIndex = find(~cellfun('isempty', IndexC));
                            splitLine = strsplit(resultCStr{lineIndex,1},' ');
                            pointFramePositions(:,m) = str2double(splitLine(3:end))';
                            
                            IndexC = strfind(gtCStr, strcat({'VERTEX_POINT_3D'},{' '},...
                                {num2str(objectFramePoints(m))},{' '}));
                            lineIndex = find(~cellfun('isempty', IndexC));
                            splitLine = strsplit(gtCStr{lineIndex,1},' ');
                            pointFrameGTPositions(:,m) = str2double(splitLine(3:end))';
                        end
                        centroid = mean(pointFramePositions,2);
                        gtCentroid = mean(pointFrameGTPositions,2);
                        speed = norm(motionValue(1:3) - (eye(3)-rot(motionValue(4:6)))*centroid);
                        gtSpeed = norm(gtMotionValue(1:3) - (eye(3)-rot(gtMotionValue(4:6)))*gtCentroid);
                        currentObjectSpeedErrors = [currentObjectSpeedErrors, abs(gtSpeed-speed)/gtSpeed];
                    end
                end
            end
            averageObjectSpeedErrors(1,j) = mean(currentObjectSpeedErrors);
        end
        objectSpeedErrors{i} = averageObjectSpeedErrors;
    end    
end

if ~constantMotionAssumption
    averageObjectSpeedErrors = zeros(1,length(objectIndexCell));
    %% wrong!!
    for i=1:min(length(objectIndexCell),length(gtObjectMotions))    
        averageObjectSpeedError = objectSpeedErrors{i};
        averageObjectSpeedErrors(1,i) = mean(averageObjectSpeedError);
    end
end
speedError = mean(averageObjectSpeedErrors);

end