function writeGroundtruthGraphFile(filePath,unique3DPoints)

% *VERTICES
% VERTEX_POINT_3D (point)
% VERTEX_POSE_LOG_SE3 (pose)
% 2POINTS_DataAssociation (point data association)

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

vertexID = 1;
cameraVertexID = [];
pointVertexID = [];
pointLabelID = [];
pointsSeenSoFar = [];
staticPointSeen = 0;
staticPointWritten = 0;

for i = 1:28
    % write a pose
    camID = i;
    fid = fopen(strcat(filePath,'cameraGroundtruth.txt'));
    line = textscan(fid,'%s',1,'delimiter','\n','headerlines',camID-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    pose = str2double(splitLine(3:end));
    fclose(fid);
    
    if(isempty(cameraVertexID))
        cameraVertexID = [cameraVertexID; camID, vertexID];
        vertexID = vertexID + 1;
    elseif (~any(cameraVertexID(:,1)==camID))
        cameraVertexID = [cameraVertexID; camID, vertexID];
        vertexID = vertexID + 1;
    end
    fid = fopen(strcat(filePath,'GT_GraphFile.graph'),'a');
    fprintf(fid,'%s %d %6f %6f %6f %6f %6f %6f','VERTEX_POSE_LOG_SE3',...
        cameraVertexID(cameraVertexID(:,1)==camID,2),pose);
    fprintf(fid,'\n');
    fclose(fid);
    
    % write all points seen from pose i
    [row,~] = find(camerasLabelsPoints(:,1)==camID);
    for j = 1:length(row)
        pointLabel = camerasLabelsPoints(row(j),2);
        pointID = camerasLabelsPoints(row(j),3);
        if pointLabel == 5
            if ~staticPointSeen
            pointVertexID = [pointVertexID ;pointID,vertexID];
            pointLabelID = [pointLabelID;pointLabel,vertexID];
            vertexID = vertexID + 1;
            end
            staticPointSeen = 1;
        elseif isempty(pointVertexID) && pointLabel~=5
            pointVertexID = [pointVertexID ;pointID,vertexID];
            pointLabelID = [pointLabelID;pointLabel,vertexID];
            vertexID = vertexID + 1;
        elseif ~any(pointVertexID(:,1)==pointID) && pointLabel~=5
            pointVertexID = [pointVertexID ;pointID,vertexID];
            pointLabelID = [pointLabelID;pointLabel,vertexID];
            vertexID = vertexID + 1;
        end
        % static point with label 5 should be written only once
        if isempty(pointsSeenSoFar) && pointLabel~=5
            pointsSeenSoFar = [pointsSeenSoFar, pointID];
            fid = fopen(strcat(filePath,'GT_GraphFile.graph'),'a');
            fprintf(fid,'%s %d %6f %6f %6f','VERTEX_POINT_3D',...
                pointVertexID(pointVertexID(:,1)==pointID,2),...
                unique3DPoints(2:end,pointID)');
            fprintf(fid,'\n');
            fclose(fid);
        elseif ~any(pointsSeenSoFar == pointID) && pointLabel~=5
            pointsSeenSoFar = [pointsSeenSoFar, pointID];
            fid = fopen(strcat(filePath,'GT_GraphFile.graph'),'a');
            fprintf(fid,'%s %d %6f %6f %6f','VERTEX_POINT_3D',...
                pointVertexID(pointVertexID(:,1)==pointID,2),...
                unique3DPoints(2:end,pointID)');
            fprintf(fid,'\n');
            fclose(fid);    
        elseif pointLabel == 5
            idx = find(unique3DPoints(1,:)==pointLabel);
            if ~staticPointWritten
                pointsSeenSoFar = [pointsSeenSoFar, pointID];
                fid = fopen(strcat(filePath,'GT_GraphFile.graph'),'a');
                fprintf(fid,'%s %d %6f %6f %6f','VERTEX_POINT_3D',...
                    pointVertexID(pointVertexID(:,1)==pointID,2),...
                    unique3DPoints(2:end,idx(1))');
                fprintf(fid,'\n');
                fclose(fid);
            end
            staticPointWritten = 1;
        end
        
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
        fid = fopen(strcat(filePath,'GT_GraphFile.graph'),'a');
        fprintf(fid,'%s %d %d %d','2POINTS_DataAssociation',point1VertexID,...
            point2VertexID, objID);
        fprintf(fid,'\n');
        fclose(fid);
    end
end
save('cameraVertexID','cameraVertexID');
save('pointVertexID','pointVertexID');
save('pointLabelID','pointLabelID');
    
end