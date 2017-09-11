function writeMeasurementsGraphFile(filePath)

% * EDGES
% EDGE_LOG_SE3 (odometry)
% EDGE_3D (point observation)
% 2POINTS_DataAssociation (point data association)

cameraVertexID = load('/home/mina/workspace/src/Git/do-slam/cameraVertexID.mat');
cameraVertexID = cameraVertexID.cameraVertexID;
pointVertexID  = load('/home/mina/workspace/src/Git/do-slam/pointVertexID.mat');
pointVertexID  = pointVertexID.pointVertexID;
pointLabelID  = load('/home/mina/workspace/src/Git/do-slam/pointLabelID.mat');
pointLabelID  = pointLabelID.pointLabelID;

% preparation
fid = fopen(strcat(filePath,'landmarkMeasGraphFile.txt'));
camerasLabelsPoints = [];
while (~feof(fid))
    tline = fgetl(fid);
    a = textscan(tline,'%s %d %d %d %f %f %f %f %f %f %f %f %f','delimiter',' ');
    camerasLabelsPoints = [camerasLabelsPoints;cell2mat(a(2)),...
        cell2mat(a(3)),cell2mat(a(4))];
end
fclose(fid);

fid = fopen(strcat(filePath,'landmarkMeasGraphFile.txt'));
a = textscan(fid,'%s %d %d %d %f %f %f %f %f %f %f %f %f','delimiter',' ');
b = cell2mat(a(5:end));
fclose(fid);

fid = fopen(strcat(filePath,'odometryMeasGraphFile.txt'));
format = repmat('%s ',1,31);
g = textscan(fid,format,'delimiter',' ');
fclose(fid);
for i = 5:size(g,2)
    h(:,i-4) = str2double(cellstr(g{i}));
end


pointsSeenSoFar = [];

for i = 1:28
    % write 3D points measurements from pose i
    camID = i;
    [row,~] = find(camerasLabelsPoints(:,1)==camID);
    for j = 1:length(row)
        line = camerasLabelsPoints(row(j),3);
        pointMeas = b(line,1:3);
        pointMeasCov = b(line,4:end);
        pointID = camerasLabelsPoints(row(j),3);
        assert(~isempty(cameraVertexID(cameraVertexID(:,1)==camID,2)))
        assert(~isempty(pointVertexID(pointVertexID(:,1)==pointID,2)))
        fid = fopen(strcat(filePath,'icra18Measurement_GraphFile.graph'),'a');
        fprintf(fid,'%s %d %d %6f %6f %6f %6f %6f %6f %6f %6f %6f','EDGE_3D',...
            cameraVertexID(cameraVertexID(:,1)==camID,2),...
            pointVertexID(pointVertexID(:,1)==pointID,2),pointMeas, pointMeasCov);
        fprintf(fid,'\n');
        fclose(fid);
        pointsSeenSoFar = [pointsSeenSoFar, pointID];
    end
    
    
    % write odometry
    lastReachedFrame = 28;
    if(i~=lastReachedFrame)
        pose1ID = i;
        pose2ID = i+1;
        line = i;
        odometryMeas = h(line,1:6);
        odometryMeasCov = h(line,7:end);
        assert(~isempty(cameraVertexID(cameraVertexID(:,1)==pose1ID,2)))
        assert(~isempty(cameraVertexID(cameraVertexID(:,1)==pose2ID,2)))
        fid = fopen(strcat(filePath,'icra18Measurement_GraphFile.graph'),'a');
        format =  strcat('%s %d %d',repmat(' %6f',1,6),repmat(' %6f',1,21));
        fprintf(fid,format,'EDGE_R3_SO3',...
            cameraVertexID(cameraVertexID(:,1)==pose1ID,2),...
            cameraVertexID(cameraVertexID(:,1)==pose2ID,2),...
            odometryMeas,odometryMeasCov);
        fprintf(fid,'\n');
        fclose(fid);
    end 
end

% write 2 points data association
labels = unique(pointLabelID,'rows');
uniqueLabels = unique(labels(:,1),'rows','stable');
for j =1:size(uniqueLabels,1)
    idx = find(labels(:,1) == uniqueLabels(j));
    switch uniqueLabels(j)
        case {1,2}
            objID = 1;
        case {3,4}
            objID = 2;
        case 5
            objID = 0;
    end
    for k=1:length(idx)-1
        point1VertexID = labels(idx(k),2);
        point2VertexID = labels(idx(k+1),2);
        fid = fopen(strcat(filePath,'icra18Measurement_GraphFile.graph'),'a');
        fprintf(fid,'%s %d %d %d','2POINTS_DataAssociation',point1VertexID,...
            point2VertexID, objID);
        fprintf(fid,'\n');
        fclose(fid);
    end
end


end