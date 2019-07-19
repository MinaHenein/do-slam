function variableWindowShift(gtGraphFilePath,measGraphFilePath,globalObjectsGraphFileIndx,windowSize,shiftSize)

assert(shiftSize <= windowSize);
if windowSize ~= inf
    overlap = windowSize - shiftSize;
end

fid = fopen(gtGraphFilePath, 'r');
gtData = textscan(fid,'%s','Delimiter','\n');
gtCStr = gtData{1};
fclose(fid);

fid = fopen(measGraphFilePath, 'r');
measData = textscan(fid,'%s','Delimiter','\n');
measCStr = measData{1};
fclose(fid);

% globalObjectsGraphFileIndx: 
% global object tracking id, frame number, graph vertex number
objectIds = {};
for i = 1:size(globalObjectsGraphFileIndx,1)
    if isempty(objectIds)
        objectIds{1,1} = globalObjectsGraphFileIndx(i,1);
        objectIds{1,2} = globalObjectsGraphFileIndx(i,3);
    else
        indx = find([objectIds{:,1}] == globalObjectsGraphFileIndx(i,1));
        if ~isempty(indx)
            objectIds{indx,2} = [objectIds{indx,2}, globalObjectsGraphFileIndx(i,3)];
        else
            objectIds{end+1,1} = globalObjectsGraphFileIndx(i,1);
            objectIds{end,2} = globalObjectsGraphFileIndx(i,3);
        end
    end
end
newObjectVertices = zeros(size(objectIds,1),1);
for i = 1:size(objectIds,1)
    allObjectVertices = objectIds{i,2};
    newObjectVertices(i,1) = allObjectVertices(1);
end

% produce constant motion graph files
% GT file
gtCStrCopy = gtCStr;
IndexC = strfind(gtCStrCopy, 'VERTEX_SE3Motion');
Index = find(not(cellfun('isempty',IndexC)));
for i = 1:length(Index)
    line = strsplit(gtCStr{Index(i),1});
    vertexNumber = str2double(line(2));
    if ~ismember(vertexNumber, newObjectVertices)
        indx = find(globalObjectsGraphFileIndx(:,3) == vertexNumber);
        objectGlobalId = globalObjectsGraphFileIndx(indx,1);
        objectVertices = objectIds{[objectIds{:,1}] == objectGlobalId,2};
        newLine = line;
        newLine{1,2} = num2str(objectVertices(1));
        gtCStrCopy{Index(i)} = char(join(newLine));
    end
    if windowSize == inf
        % Save in a different file
        filepath = strcat(gtGraphFilePath(1:end-6),'_constantMotion.graph');
        fileID = fopen(filepath,'w');
        fprintf(fileID, '%s\n', gtCStrCopy{:});
        fclose(fileID);
    end
end
% meas file
measCStrCopy = measCStr;
IndexC = strfind(measCStrCopy, '2PointsDataAssociation');
Index = find(not(cellfun('isempty',IndexC)));
for i = 1:length(Index)
    line = strsplit(measCStr{Index(i),1});
    vertexNumber = str2double(line(4));
    if ~ismember(vertexNumber, newObjectVertices)
        indx = find(globalObjectsGraphFileIndx(:,3) == vertexNumber);
        objectGlobalId = globalObjectsGraphFileIndx(indx,1);
        objectVertices = objectIds{[objectIds{:,1}] == objectGlobalId,2};
        newLine = line;
        newLine{1,4} = num2str(objectVertices(1));
        measCStrCopy{Index(i)} = char(join(newLine));
    end
    if windowSize == inf
        % Save in a different file
        filepath = strcat(measGraphFilePath(1:end-6),'_constantMotion.graph');
        fileID = fopen(filepath,'w');
        fprintf(fileID, '%s\n', measCStrCopy{:});
        fclose(fileID);
        return
    end
end

% produce variable window, shift size graph files
% GT file
gtCStrCopy2 = gtCStrCopy;
objectWindows = zeros(size(objectIds,1),1);
IndexC = strfind(gtCStrCopy2, 'VERTEX_SE3Motion');
Index = find(not(cellfun('isempty',IndexC)));
nVertices = size(gtCStr,1);
objectIds = newObjectVertices;
for i = 1:length(Index)
    line = strsplit(gtCStrCopy2{Index(i),1});
    vertexNumber = str2double(line(2));
    indx = find(objectIds == vertexNumber);
    objectWindows(indx,1) = objectWindows(indx,1) + 1;
    
    isObjectWithinWindow = (objectWindows(indx,1) < windowSize);
    if ~isObjectWithinWindow
        % new vertex
        nVertices = nVertices + 1;
        objectVertex = nVertices;
        newLine = line;
        newLine{1,2} = num2str(objectVertex);
        gtCStrCopy2{Index(i)} = char(join(newLine));
        
        for j = i+1:length(Index)
            line = strsplit(gtCStrCopy2{Index(j),1});
            objectVertexNumber = str2double(line(2));
            if objectVertexNumber == vertexNumber
                newLine = line;
                newLine{1,2} = num2str(objectVertex);
                gtCStrCopy2{Index(j)} = char(join(newLine));
            end
        end
        
        % new objectWindows entry with 0 value
        objectIds = [objectIds; objectVertex];
        objectWindows(end+1, 1) = 1;
    end
    
end
% Save in a different file
filepath = strcat(gtGraphFilePath(1:end-6),'_window',num2str(windowSize),'.graph');
fileID = fopen(filepath,'w');
fprintf(fileID, '%s\n', gtCStrCopy2{:});
fclose(fileID);

% meas file
measCStrCopy2 = measCStrCopy;
%% to be implemented


end