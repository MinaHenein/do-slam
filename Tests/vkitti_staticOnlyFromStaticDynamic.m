clear; clc;
% Measurements
%vKitti_dynamicStaticMeas_1_v5
%occlusionWorkingMeas_v1
filepath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/finalNoiseSequence0002_Meas.graph';
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);

poses = [];
points = [];
dynamicPoints = [];
pointSeenByCamera = {};

for i =1:size(CStr,1)
line = CStr{i};
splitLine = strsplit(line,' ');
    if strcmp(line(1:length('EDGE_3D')),'EDGE_3D')
        camID = str2double(cell2mat(splitLine(2)));
        pointID = str2double(cell2mat(splitLine(3)));
        poses = [poses; camID];
        points = [points; pointID];
        pointSeenByCamera{camID, end+1} = pointID;
    end
    if strcmp(line(1:length('EDGE_R3_SO3')),'EDGE_R3_SO3')
        cam1ID = str2double(cell2mat(splitLine(2)));
        cam2ID = str2double(cell2mat(splitLine(3)));
        poses = [poses; cam1ID];
        poses = [poses; cam2ID];
    end
    if strcmp(line(1:length('2POINTS_DataAssociation')),'2POINTS_DataAssociation')
        dynamicPoint1ID = str2double(cell2mat(splitLine(2)));
        dynamicPoint2ID = str2double(cell2mat(splitLine(3)));
        dynamicPoints = [dynamicPoints; dynamicPoint1ID];
        dynamicPoints = [dynamicPoints; dynamicPoint2ID];
    end
end

poses = unique(poses);
points = unique(points);
dynamicPoints = unique(dynamicPoints);

%vKitti_OcclusionWorkingMeas_staticOnlyTest
fileToWritepath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/finalNoiseSequence0002_staticOnlyMeas.graph';
fileToWriteID = fopen(fileToWritepath,'w');

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
                IndexC = strfind(CStr, strcat({'EDGE_3D'},{' '},{num2str(poses(i))},...
                    {' '},{num2str(pointSeen(j))},{' '}));
                lineIndex = find(~cellfun('isempty', IndexC));
                fileID = fopen(filepath,'r');
                line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                line = cell2mat(line{1,1});
                splitLine = str2double(strsplit(line,' '));
                edgeValue = splitLine(1,4:6);
                edgeCovariance = splitLine(1,7:end);
                fclose(fileID);
                writeEdge('EDGE_3D',poses(i),pointSeen(j),edgeValue,edgeCovariance,fileToWriteID);
            end
        end
    end
        if i<length(poses)
            IndexC = strfind(CStr, strcat({'EDGE_R3_SO3'},{' '},{num2str(poses(i))},...
                {' '},{num2str(poses(i+1))},{' '}));
            lineIndex = find(~cellfun('isempty', IndexC));
            fileID = fopen(filepath,'r');
            line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
            line = cell2mat(line{1,1});
            splitLine = str2double(strsplit(line,' '));
            edgeValue = splitLine(1,4:9);
            edgeCovariance = splitLine(1,10:end);
            fclose(fileID);
            writeEdge('EDGE_R3_SO3',poses(i),poses(i+1),edgeValue,edgeCovariance,fileToWriteID);
        end
end
fclose(fileToWriteID);

% GT
%vKitti_dynamicStaticGT_1_v5
%occlusionWorking_v1
filepath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/finalNoiseSequence0002_GT.graph';
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);

%vKitti_OcclusionWorkingGT_staticOnlyTest
fileToWritepath = '/home/mina/workspace/src/Git/do-slam/Data/GraphFiles/finalNoiseSequence0002_staticOnlyGT.graph';
fileToWriteID = fopen(fileToWritepath,'w');

pointWritten = [];
for i=1:length(poses)
    IndexC = strfind(CStr, strcat({'VERTEX_POSE_R3_SO3'},{' '},{num2str(poses(i))},{' '}));
    lineIndex = find(~cellfun('isempty', IndexC));
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
    line = cell2mat(line{1,1});
    splitLine = str2double(strsplit(line,' '));
    vertexValue = splitLine(1,3:8);
    fclose(fileID);
    writeVertex('VERTEX_POSE_R3_SO3',poses(i),vertexValue,fileToWriteID);
    if ~isempty(cell2mat(pointSeenByCamera(poses(i),:)))
        pointSeen = cell2mat(pointSeenByCamera(poses(i),:));
        for j=1:length(pointSeen)
            if ~ismember(pointSeen(j),dynamicPoints)
                if isempty(pointWritten) || ~ismember(pointSeen(j),pointWritten)
                    IndexC = strfind(CStr, strcat({'VERTEX_POINT_3D'},{' '},{num2str(pointSeen(j))},{' '}));
                    lineIndex = find(~cellfun('isempty', IndexC));
                    fileID = fopen(filepath,'r');
                    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
                    line = cell2mat(line{1,1});
                    splitLine = str2double(strsplit(line,' '));
                    vertexValue = splitLine(1,3:5);
                    fclose(fileID);
                    writeVertex('VERTEX_POINT_3D',pointSeen(j),vertexValue,fileToWriteID);
                    pointWritten = [pointWritten,pointSeen(j)];
                end
            end
        end
        
    end
end
fclose(fileToWriteID);