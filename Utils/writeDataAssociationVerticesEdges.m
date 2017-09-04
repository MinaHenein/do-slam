function writeDataAssociationVerticesEdges(config,constantSE3ObjectMotion)

GTFileName = config.groundTruthFileName;
fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find first line with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
nLinesAdded = 0;

for j=1:length(Index)
    % get line of Index
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)+nLinesAdded-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    fclose(fileID);
    % get value for vertices that will constitute new edge
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
    line1 = textscan(fileID, '%s', 1, 'delimiter', '\n', 'headerlines', index1-1);
    line1 = cell2mat(line1{1,1});
    if ~strcmp(line1(1:length('VERTEX_POINT_3D')),'VERTEX_POINT_3D')
        error('vertex is not a 3D point');
    end
    splitLine1 = str2double(strsplit(line1,' '));
    vertex1Value = splitLine1(1,3:5)';
    fclose(fileID);
    
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
    line2 = textscan(fileID, '%s', 1, 'delimiter', '\n', 'headerlines', index2-1);
    line2 = cell2mat(line2{1,1});
    if ~strcmp(line2(1:length('VERTEX_POINT_3D')),'VERTEX_POINT_3D')
        error('vertex is not a 3D point');
    end
    splitLine2 = str2double(strsplit(line2,' '));
    vertex2Value = splitLine2(1,3:5)';
    fclose(fileID);
    
    % count number of vertices
    nVertices = 0;
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
    line = fgetl(fileID);
    while ischar(line)
        if strcmp(line(1:length('VERTEX')),'VERTEX')
            nVertices = nVertices+1;
        end
        line = fgetl(fileID);
    end
    fclose(fileID);
    isNewSE3Vertex = 0;
    IndexC = strfind(CStr, 'VERTEX_SE3Motion');
    SE3MotionIndex = find(~cellfun('isempty', IndexC),1);
    if isempty(SE3MotionIndex)
        isNewSE3Vertex = 1;
        newVertexID = nVertices+1;
    else
        fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
            config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
        line = textscan(fileID, '%s', 1, 'delimiter', '\n', 'headerlines',...
            SE3MotionIndex-1);
        line = cell2mat(line{1,1});
        if ~strcmp(line(1:length('VERTEX_SE3Motion')),'VERTEX_SE3Motion')
            error('vertex is not SE3 motion veretx');
        end
        splitLine = str2double(strsplit(line,' '));
        lastSE3VertexValue = splitLine(1,3:8)';
        fclose(fileID);
    end
    % get motion model and replace line accordingly
    if ~isempty(Index(j)+nLinesAdded)
        switch config.motionModel
            case 'constantSE3MotionDA'
                vertex = struct();
                vertex.label = config.SE3MotionVertexLabel;
                vertex.index = newVertexID;
                vertex.value = [constantSE3ObjectMotion(1:3,4);...
                    arot(constantSE3ObjectMotion(1:3,1:3))];
                
                edge = struct();
                edge.index1 = index1;
                edge.index2 = index2;
                edge.index3 = newVertexID;
                edge.label = config.pointSE3MotionEdgeLabel;
                edge.value = vertex1Value -...
                    (constantSE3ObjectMotion(1:3,1:3)'*...
                    vertex2Value -...
                    constantSE3ObjectMotion(1:3,1:3)'*constantSE3ObjectMotion(1:3,4));
                edge.std = config.std2PointsSE3Motion;
                edge.cov = config.cov2PointsSE3Motion;
                edge.covUT = covToUpperTriVec(edge.cov);
            case 'constantSE3' % to be done
            case 'constantSE3Motion' % to be done
        end
        
        if  isNewSE3Vertex
            % Shift all lines below vertices 1 line below to allow space for 
            % new vertex in the right place
            fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
                config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
            fseek(fileID, 0, 'eof');
            fileSize = ftell(fileID);
            frewind(fileID);
            data = fread(fileID, fileSize, 'uint8');
            nLines = sum(data == 10) + 1;
            fclose(fileID);
            for i= nLines-1:-1:nVertices+1
                CStr(i+1) = CStr(i);
            end
        end
        if ~(isempty(SE3MotionIndex))
            if lastSE3VertexValue~=vertex.value
                isNewSE3Vertex=1;
            end
        end
        if isNewSE3Vertex
            CStr(newVertexID) = cellstr(sprintf('%s %d %f %f %f %f %f %f',...
            vertex.label,vertex.index,vertex.value'));
            % if new line added before 2POINTS_DataAssociation, incerment
            % nLinesAdded to be able to delete the correct line afterwards
            nLinesAdded = nLinesAdded+1;
        end
        if isfield(edge, 'index3')
            CStr(end+1) = cellstr(sprintf('%s %d %d %d %f %f %f %f %f %f %f %f %f',...
                edge.label,edge.index1,edge.index2,edge.index3,edge.value',...
                edge.covUT));
        end
        CStr(Index(j)+nLinesAdded) = [];
        nLinesAdded = nLinesAdded-1;
    end
    
    % Save the file again:
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName), 'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end

MeasurementsFileName = config.measurementsFileName;
fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find first line with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
nLinesAdded = 0;

for j=1:length(Index)
    % get line of Index
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName),'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',...
        Index(j)+nLinesAdded-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    fclose(fileID);
    
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
    Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
    CStrGT = Data{1};
    fclose(fileID);
    searchedStr = strcat({'EDGE_2POINTS_SE3Motion'},{' '},{num2str(index1)},...
        {' '},{num2str(index2)});
    IndexC = strfind(CStrGT, searchedStr);
    % find first line with a DataAssociation entry
    edgeIndex = find(~cellfun('isempty', IndexC),1);
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
    line = textscan(fileID, '%s', 1, 'delimiter', '\n', 'headerlines', edgeIndex-1);
    line = cell2mat(line{1,1});
    if ~strcmp(line(1:length('EDGE_2POINTS_SE3Motion')),'EDGE_2POINTS_SE3Motion')
        error('edge is not EDGE_2POINTS_SE3Motion');
    end
    splitLine = str2double(strsplit(line,' '));
    index3 = splitLine(1,4);
    valueEdge = splitLine(1,5:7)';
    muEdge =  zeros(size(valueEdge,1),1);
    sigmaEdge = config.std2PointsSE3Motion;
    covEdge = config.cov2PointsSE3Motion;
    edgeCovUT = covToUpperTriVec(covEdge);
    edgeLabel = config.pointSE3MotionEdgeLabel;
    edgeValue = addGaussianNoise(config,muEdge,sigmaEdge,valueEdge);
    fclose(fileID);
    CStr(end+1) = cellstr(sprintf('%s %d %d %d %f %f %f %f %f %f %f %f %f',...
        edgeLabel,index1,index2,index3,edgeValue',edgeCovUT));

    CStr(Index(j)+nLinesAdded) = [];
    nLinesAdded = nLinesAdded-1;
     % Save the file again:
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName), 'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);

end

end

