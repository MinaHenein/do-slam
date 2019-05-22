function [translationError, rotationError, speedError] = ...
    computeObjectMotionError(resultFileName, gtFileName, objectPosesMatrix, constantMotionAssumption)

if constantMotionAssumption
    averageGTObjectMotions = vKitti_objectMotionAveraged(objectPosesMatrix);
else
    [gtObjectMotions,~] = vKitti_objectMotion(objectPosesMatrix);
end

gtFilepath = strcat(pwd,'/Data/GraphFiles/',gtFileName);
gtFileID = fopen(gtFilepath,'r');
gtData = textscan(gtFileID, '%s', 'delimiter', '\n', 'whitespace', '');
gtCStr = gtData{1};
fclose(gtFileID);
if constantMotionAssumption
    searchedStr = '2PointsDataAssociation';
else
    searchedStr = '2OBJECTS_DataAssociation';
end
IndexC = strfind(gtCStr, searchedStr);
Index = find(~cellfun('isempty', IndexC));

if constantMotionAssumption
    objectIndexCell = {};
    for i=1:length(Index)
        splitLine = strsplit(gtCStr{Index(i),1},' ');
        index = str2double(splitLine{1,4});
        objectIndexCell{i,1} = index;
    end
else
    objectIndices = zeros(length(Index),2);
    for i=1:length(Index)
        splitLine = strsplit(gtCStr{Index(i),1},' ');
        index1 = str2double(splitLine{1,2});
        index2 = str2double(splitLine{1,3});
        objectIndices(i,:) = [index1, index2];
    end
    objectIndexCell = {};
    for i=1:size(objectIndices,1)
        if i==1
            objectIndexCell{1} = [objectIndices(1,1), objectIndices(1,2)];
        else
            objectIndex = objectIndices(i,1);
            if ismember(objectIndex,[objectIndices(1:i-1,:)])
                for j=1:length(objectIndexCell)
                    if ~isempty(find([objectIndexCell{j,:}] == objectIndex))
                        break
                    end
                end
                objectIndexCell{j} = [objectIndexCell{j}, objectIndices(i,2)];
            else
                objectIndexCell{end+1,1} = [objectIndices(i,1), objectIndices(i,2)];
            end
        end
    end
end


resultFilepath = strcat(pwd,'/Data/GraphFiles/',resultFileName);
resultFileID = fopen(resultFilepath,'r');
resultData = textscan(resultFileID, '%s', 'delimiter', '\n', 'whitespace', '');
resultCStr = resultData{1};
fclose(resultFileID);

if constantMotionAssumption
    averageSE3ObjectMotions = zeros(6,length(objectIndexCell));
    for i=1:length(objectIndexCell)
        currentObjectIndex = objectIndexCell{i};
        IndexC = strfind(resultCStr, strcat({'VERTEX_SE3Motion'},{' '},...
            {num2str(currentObjectIndex)},{' '}));
        lineIndex = find(~cellfun('isempty', IndexC));
        splitLine = strsplit(resultCStr{lineIndex,1},' ');
        value = splitLine(3:end);
        averageSE3ObjectMotions(:,i) = value';
    end
else
    objectMotions = {};
    for i=1:length(objectIndexCell)
        currentObjectMotionIndices = [objectIndexCell{i}];
        currentObjectMotions =  zeros(6,length(currentObjectMotionIndices));
        for j = 1:length(currentObjectMotionIndices)
            IndexC = strfind(resultCStr, strcat({'VERTEX_SE3Motion'},{' '},...
                {num2str(currentObjectMotionIndices(j))},{' '}));
            lineIndex = find(~cellfun('isempty', IndexC));
            if ~isempty(lineIndex)
                splitLine = strsplit(resultCStr{lineIndex,1},' ');
                value = str2double(splitLine(3:end));
                currentObjectMotions(:,j) = value';
            end
        end
        objectMotions{i} = currentObjectMotions;
    end    
end

if constantMotionAssumption
    objectMotionError = zeros(6,length(objectIndexCell));
    for i=1:length(objectIndexCell)
        error = AbsoluteToRelativePoseR3xso3(averageSE3ObjectMotions(:,i),averageGTObjectMotions(:,i));
        objectMotionError(:,i) = error;
    end
else
    objectMotionError = {};
    for i=1:length(objectIndexCell)
        currentObjectMotions = objectMotions{i};
        currentGTObjectMotions = gtObjectMotions{i};
        currentObjectMotionError =  zeros(6,size(currentObjectMotions,2));
        for j = 1:size(currentObjectMotions,2)
            error = AbsoluteToRelativePoseR3xso3(currentObjectMotions(:,j),currentGTObjectMotions(:,j));
            currentObjectMotionError(:,j) = error;
        end
        objectMotionError{i} = currentObjectMotionError;
    end
end

averageObjectMotionTranslationError = zeros(1,length(objectIndexCell));
averageObjectMotionRotationError = zeros(1,length(objectIndexCell));
if constantMotionAssumption
    for i=1:size(objectMotionError,2)
        error = objectMotionError(:,i);
        averageObjectMotionTranslationError(1,i) = norm(error(1:3));
        averageObjectMotionRotationError(1,i) = norm(error(4:6));
    end
else
    for i=1:length(objectIndexCell)
        currentObjectMotionErros = objectMotionError{i};
        objectTranslationError = zeros(1,size(currentObjectMotionErros,2));
        objectRotationError = zeros(1,size(currentObjectMotionErros,2));
        for j = 1:size(currentObjectMotionErros,2)
            error = currentObjectMotionErros(:,j);
            objectTranslationError(1,j) = norm(error(1:3));
            objectRotationError(1,j) = norm(error(4:6));
        end
        averageObjectMotionTranslationError(1,i) = mean(objectTranslationError);
        averageObjectMotionTranslationError(1,i) = mean(objectRotationError);
    end
end
translationError = mean(averageObjectMotionTranslationError);
rotationError = mean(averageObjectMotionRotationError);

end