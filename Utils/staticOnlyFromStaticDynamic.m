function staticOnlyFromStaticDynamic(config, gtFilePath, measFilePath, initFilePath)

% gt
gtFileID = fopen(gtFilePath,'r');
gtData = textscan(gtFileID, '%s', 'delimiter', '\n', 'whitespace', '');
gtCStr = gtData{1};
fclose(gtFileID);

% Measurements
measFileID = fopen(measFilePath,'r');
measData = textscan(measFileID, '%s', 'delimiter', '\n', 'whitespace', '');
measCStr = measData{1};
fclose(measFileID);


% init
initFileID = fopen(initFilePath,'r');
initData = textscan(initFileID, '%s', 'delimiter', '\n', 'whitespace', '');
initCStr = initData{1};
fclose(initFileID);

poses = [];
points = [];
dynamicPoints = [];
objMotions = [];
pointSeenByCamera = {};

for i =1:size(measCStr,1)
    line = measCStr{i};
    splitLine = strsplit(line,' ');
    % EDGE_3D or EDGE_2D_PIXEL
    if strcmp(line(1:length(config.posePointEdgeLabel)),config.posePointEdgeLabel)
        camID = str2double(cell2mat(splitLine(2)));
        pointID = str2double(cell2mat(splitLine(3)));
        poses = [poses; camID];
        points = [points; pointID];
        pointSeenByCamera{camID, end+1} = pointID;
    end
    %EDGE_R3_SO3
    if strcmp(line(1:length(config.posePoseEdgeLabel)),config.posePoseEdgeLabel)
        cam1ID = str2double(cell2mat(splitLine(2)));
        cam2ID = str2double(cell2mat(splitLine(3)));
        poses = [poses; cam1ID];
        poses = [poses; cam2ID];
    end
    % EDGE_2POINTS_SE3Motion
    if strcmp(line(1:length(config.pointSE3MotionEdgeLabel)),config.pointSE3MotionEdgeLabel)
        dynamicPoint1ID = str2double(cell2mat(splitLine(2)));
        dynamicPoint2ID = str2double(cell2mat(splitLine(3)));
        objectMotionID = str2double(cell2mat(splitLine(4)));
        dynamicPoints = [dynamicPoints; dynamicPoint1ID];
        dynamicPoints = [dynamicPoints; dynamicPoint2ID];
        objMotions = [objMotions; objectMotionID];
    end
end

poses = unique(poses);
points = unique(points);
dynamicPoints = unique(dynamicPoints);
objMotions = unique(objMotions);
%**************************************************************************
measFileToWritePath = strcat(measFilePath(1:end-6),'_staticOnly.graph');
measFileToWriteID = fopen(measFileToWritePath,'w');

if size(pointSeenByCamera,1) < poses(end)
    for i = size(pointSeenByCamera,1)+1:poses(end)
        pointSeenByCamera{i, end+1} = [];
    end
end

for i=1:length(poses)
    if ~isempty(cell2mat(pointSeenByCamera(poses(i),:)))
        pointSeen = cell2mat(pointSeenByCamera(poses(i),:));
        for j=1:length(pointSeen)
            if ~ismember(pointSeen(j),dynamicPoints)
                IndexC = strfind(measCStr, strcat({config.posePointEdgeLabel},{' '},{num2str(poses(i))},...
                    {' '},{num2str(pointSeen(j))},{' '}));
                lineIndex = find(~cellfun('isempty', IndexC));
                fileID = fopen(measFilePath,'r');
                line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                line = cell2mat(line{1,1});
                splitLine = str2double(strsplit(line,' '));
                edgeValue = splitLine(1,4:6);
                edgeCovariance = splitLine(1,7:end);
                fclose(fileID);
                writeEdge(config.posePointEdgeLabel,poses(i),pointSeen(j),edgeValue,edgeCovariance,measFileToWriteID);
            end
        end
    end
        if i<length(poses)
            IndexC = strfind(measCStr, strcat({config.posePoseEdgeLabel},{' '},{num2str(poses(i))},...
                {' '},{num2str(poses(i+1))},{' '}));
            lineIndex = find(~cellfun('isempty', IndexC));
            fileID = fopen(measFilePath,'r');
            line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
            line = cell2mat(line{1,1});
            splitLine = str2double(strsplit(line,' '));
            edgeValue = splitLine(1,4:9);
            edgeCovariance = splitLine(1,10:end);
            fclose(fileID);
            writeEdge(config.posePoseEdgeLabel,poses(i),poses(i+1),edgeValue,edgeCovariance,measFileToWriteID);
        end
end
fclose(measFileToWriteID);
%**************************************************************************

gtFileToWritePath = strcat(gtFilePath(1:end-6),'_staticOnly.graph');
gtFileToWriteID = fopen(gtFileToWritePath,'w');

pointWritten = [];
for i=1:length(poses)
    IndexC = strfind(gtCStr, strcat({config.poseVertexLabel},{' '},{num2str(poses(i))},{' '}));
    lineIndex = find(~cellfun('isempty', IndexC));
    fileID = fopen(gtFilePath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
    line = cell2mat(line{1,1});
    splitLine = str2double(strsplit(line,' '));
    vertexValue = splitLine(1,3:8);
    fclose(fileID);
    writeVertex(config.poseVertexLabel,poses(i),vertexValue,gtFileToWriteID);
    if ~isempty(cell2mat(pointSeenByCamera(poses(i),:)))
        pointSeen = cell2mat(pointSeenByCamera(poses(i),:));
        for j=1:length(pointSeen)
            if ~ismember(pointSeen(j),dynamicPoints)
                if isempty(pointWritten) || ~ismember(pointSeen(j),pointWritten)
                    IndexC = strfind(gtCStr, strcat({config.pointVertexLabel},{' '},{num2str(pointSeen(j))},{' '}));
                    lineIndex = find(~cellfun('isempty', IndexC));
                    fileID = fopen(gtFilePath,'r');
                    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                    line = cell2mat(line{1,1});
                    splitLine = str2double(strsplit(line,' '));
                    vertexValue = splitLine(1,3:5);
                    fclose(fileID);
                    writeVertex(config.pointVertexLabel,pointSeen(j),vertexValue,gtFileToWriteID);
                    pointWritten = [pointWritten,pointSeen(j)];
                end
            end
        end
        
    end
end
fclose(gtFileToWriteID);
%**************************************************************************

initFileToWritePath = strcat(initFilePath(1:end-6),'_staticOnly.graph');
initFileToWriteID = fopen(initFileToWritePath,'w');

pointWritten = [];
for i=1:length(poses)
    IndexC = strfind(initCStr, strcat({config.poseVertexLabel},{' '},{num2str(poses(i))},{' '}));
    lineIndex = find(~cellfun('isempty', IndexC));
    fileID = fopen(initFilePath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
    line = cell2mat(line{1,1});
    splitLine = str2double(strsplit(line,' '));
    vertexValue = splitLine(1,3:8);
    fclose(fileID);
    writeVertex(config.poseVertexLabel,poses(i),vertexValue,initFileToWriteID);
    if ~isempty(cell2mat(pointSeenByCamera(poses(i),:)))
        pointSeen = cell2mat(pointSeenByCamera(poses(i),:));
        for j=1:length(pointSeen)
            if ~ismember(pointSeen(j),dynamicPoints)
                if isempty(pointWritten) || ~ismember(pointSeen(j),pointWritten)
                    IndexC = strfind(initCStr, strcat({config.pointVertexLabel},{' '},{num2str(pointSeen(j))},{' '}));
                    lineIndex = find(~cellfun('isempty', IndexC));
                    fileID = fopen(initFilePath,'r');
                    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                    line = cell2mat(line{1,1});
                    splitLine = str2double(strsplit(line,' '));
                    vertexValue = splitLine(1,3:5);
                    fclose(fileID);
                    writeVertex(config.pointVertexLabel,pointSeen(j),vertexValue,initFileToWriteID);
                    pointWritten = [pointWritten,pointSeen(j)];
                end
            end
        end
        
    end
end
fclose(initFileToWriteID);
%**************************************************************************

end