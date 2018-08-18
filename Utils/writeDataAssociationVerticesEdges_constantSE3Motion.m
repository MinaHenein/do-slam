function writeDataAssociationVerticesEdges_constantSE3Motion(config,constantSE3ObjectMotion)

nObjects = size(constantSE3ObjectMotion,2);

GTFileName = config.groundTruthFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName);
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
Index = find(~cellfun('isempty', IndexC));
IndexOriginal = find(cellfun('isempty',IndexC));
DataNew = Data{1}([IndexOriginal; Index]); % new data with resorted vertexes
% rewrite file with  new indexes
fileID = fopen(filepath,'w');
for i=1:numel(DataNew)
    fprintf(fileID,[DataNew{i} '\n']);
end
fclose(fileID);
% re-order data association entries by object
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace',' ');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
Index = find(~cellfun('isempty', IndexC));
DataAssociation = CStr(Index);
tempFilePath = 'Data/Temp/rearrangeVertices.txt';
tempFileID = fopen(tempFilePath,'w');
for i=1:numel(DataAssociation)
    fprintf(tempFileID,[DataAssociation{i} '\n']);
end
fclose(tempFileID);
tempFileID = fopen(tempFilePath,'r');
a = textscan(tempFileID,'%s %d %d %d','delimiter',' ');
b = cell2mat(a(2:end));
fclose(tempFileID);
c = sortrows(b,3);
CStr(Index) = [];
fileID = fopen(filepath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);
fileID = fopen(filepath,'a');
for i=1:size(c,1)
    fprintf(fileID,'%s %d %d %d \n','2POINTS_DataAssociation',...
        c(i,1),c(i,2),c(i,3));
end
fclose(fileID);
delete Data/Temp/rearrangeVertices.txt;

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
nLinesAdded = 0;

% count number of vertices
nVertices = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX')),'VERTEX')
        nVertices = nVertices+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);

SE3ObjectMotion = zeros(4,4,nObjects);
for i=1:nObjects
SE3ObjectMotion(:,:,i) = [rot(constantSE3ObjectMotion(4:6,i)),...
    constantSE3ObjectMotion(1:3,i); 0 0 0 1]; 
end
constantSE3ObjectMotion = SE3ObjectMotion;

nSE3MotionVertices = 0;
isNewSE3Vertex = 0;
object = 0;
lastObject = 0;
index2Last = 0;
nLandmarksPerMotionVertex = 0;
for j=1:1:length(Index)
    Edges = {};
    if j > 1
        lastObject = object;
        index2Last = index2;
    end
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)+nLinesAdded-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    if numel(size(constantSE3ObjectMotion)) > 2
        object = str2double(splitLine{1,4});
    end
    fclose(fileID);
    % get value for vertices that will constitute new edge
    fileID = fopen(filepath,'r');
    Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
    CStr = Data{1};
    IndexC = strfind(CStr, strcat({'VERTEX_POINT_3D'},{' '},{num2str(index1)},{' '}));
    fclose(fileID);
    lineIndex = find(~cellfun('isempty', IndexC));
    fileID = fopen(filepath,'r');
    line1 = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
    line1 = cell2mat(line1{1,1});
    splitLine1 = str2double(strsplit(line1,' '));
    vertex1Value = splitLine1(1,3:5)';
    fclose(fileID);
    
    IndexC = strfind(CStr, strcat({'VERTEX_POINT_3D'},{' '},{num2str(index2)},{' '}));
    lineIndex = find(~cellfun('isempty', IndexC));
    fileID = fopen(filepath,'r');
    line2 = textscan(fileID,'%s',1,'delimiter','\n','headerlines',lineIndex-1);
    line2 = cell2mat(line2{1,1});
    splitLine2 = str2double(strsplit(line2,' '));
    vertex2Value = splitLine2(1,3:5)';
    fclose(fileID);
    
    if nSE3MotionVertices == 0
        isNewSE3Vertex = 1;
        newVertexID = nVertices +1;
        nLandmarksPerMotionVertex = 2;
    end
    if j>1 && nObjects > 1 && object~=lastObject
        nLandmarksPerMotionVertex = config.newMotionVertexPerNLandmarks;
    end
    
    if j > 1 % && nObjects > 1 && object~=lastObject 
        if nLandmarksPerMotionVertex < config.newMotionVertexPerNLandmarks
            if index1 ~= index2Last
                nLandmarksPerMotionVertex = nLandmarksPerMotionVertex + 2;
            else
                nLandmarksPerMotionVertex = nLandmarksPerMotionVertex + 1;
            end
        else
        isNewSE3Vertex = 1;
        newVertexID = newVertexID+1;
        nLandmarksPerMotionVertex = 2;
        end
    end
    
    % get motion model and replace line accordingly
    if ~isempty(Index(j)+nLinesAdded)
        switch config.motionModel
            case 'constantSE3MotionDA'
                vertex = struct();
                vertex.label = config.SE3MotionVertexLabel;
                vertex.index = newVertexID;
                if numel(size(constantSE3ObjectMotion)) > 2
                    objectSE3Motion = constantSE3ObjectMotion(:,:,object); 
                else
                    objectSE3Motion = constantSE3ObjectMotion;
                end
                vertex.value = [objectSE3Motion(1:3,4);...
                    arot(objectSE3Motion(1:3,1:3))];
                
                edge = struct();
                edge.index1 = index1;
                edge.index2 = index2;
                edge.index3 = newVertexID;
                edge.label = config.pointSE3MotionEdgeLabel;
                edge.value = vertex1Value -...
                    (objectSE3Motion(1:3,1:3)'*...
                    vertex2Value -...
                    objectSE3Motion(1:3,1:3)'*objectSE3Motion(1:3,4));                
%                 valueEdge = [0;0;0];
%                 muEdge =  zeros(size(valueEdge,1),1);
%                 sigmaEdge = config.std2PointsSE3Motion;
%                 edge.value = addGaussianNoise(config,muEdge,sigmaEdge,valueEdge);
                edge.std = config.std2PointsSE3Motion;
                edge.cov = config.cov2PointsSE3Motion;
                edge.covUT = covToUpperTriVec(edge.cov); 
            case 'constantSE3' % to be done
            case 'constantSE3Motion' % to be done
        end
        if  isNewSE3Vertex
            % Shift all lines below vertices 1 line below to allow space for 
            % new vertex in the right place
%             fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
%                 config.sep,config.graphFileFolderName,config.sep,GTFileName),'r');
%             fseek(fileID, 0, 'eof');
%             fileSize = ftell(fileID);
%             frewind(fileID);
%             data = fread(fileID, fileSize, 'uint8');
%             nLines = sum(data == 10) + 1;
%             fclose(fileID);
%             for i= nLines-1:-1:nVertices+1
%                 CStr(i+1) = CStr(i);
%             end
            CStr(newVertexID) = cellstr(sprintf('%s %d %f %f %f %f %f %f',...
            vertex.label,vertex.index,vertex.value'));
            % if new line added before 2POINTS_DataAssociation, increment
            % nLinesAdded to be able to delete the correct line afterwards
%             nLinesAdded = nLinesAdded+1;
            nSE3MotionVertices = nSE3MotionVertices +1;
        end
        
        fileID = fopen(filepath,'r');
        Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
        Str = Data{1};
        IndexC = strfind(Str, strcat({'EDGE_2POINTS_SE3Motion'},{' '},...
            {num2str(edge.index1)},{' '},{num2str(edge.index2)},{' '},...
            {num2str(edge.index3)},{' '}));
        fclose(fileID);
        if isempty(find(~cellfun('isempty', IndexC),1))
            % write edge at end of file
            Edges(end+1) = cellstr(sprintf('%s %d %d %d %f %f %f %f %f %f %f %f %f',...
            edge.label,edge.index1,edge.index2,edge.index3,edge.value',...
            edge.covUT));
        end
        % delete 1 line with 2POINTS_DataAssociation
%         CStr(Index(j)+nLinesAdded) = [];
%         nLinesAdded = nLinesAdded-1;
        isNewSE3Vertex = 0;
    end
%     % Save in the file again
%     fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
%         config.sep,config.graphFileFolderName,config.sep,GTFileName), 'w');
%     fprintf(fileID, '%s\n', CStr{:});
%     fprintf(fileID, '%s\n', Edges{:});
%     fclose(fileID);
    
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName(1:end-6),'Test.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStr{:});
    fprintf(fileID, '%s\n', Edges{:});
    fclose(fileID);
end
newGTFileName = strcat(GTFileName(1:end-6),'Test.graph');
GTFilePath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName);
deleteDataAssociationFromGraphFile(GTFilePath)

MeasurementsFileName = config.measurementsFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% re-order data association entries by object
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
Index = find(~cellfun('isempty', IndexC));
DataAssociation = CStr(Index);
tempFilePath = 'Data/Temp/rearrangeVertices.txt';
tempFileID = fopen(tempFilePath,'w');
for i=1:numel(DataAssociation)
    fprintf(tempFileID,[DataAssociation{i} '\n']);
end
fclose(tempFileID);
tempFileID = fopen(tempFilePath,'r');
a = textscan(tempFileID,'%s %d %d %d','delimiter',' ');
b = cell2mat(a(2:end));
fclose(tempFileID);
c = sortrows(b,3);
CStr(Index) = [];
fileID = fopen(filepath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);
fileID = fopen(filepath,'a');
for i=1:size(c,1)
    fprintf(fileID,'%s %d %d %d \n','2POINTS_DataAssociation',...
        c(i,1),c(i,2),c(i,3));
end
fclose(fileID);
delete Data/Temp/rearrangeVertices.txt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
IndexOriginal = find(cellfun('isempty',IndexC));
DataNew = Data{1}([IndexOriginal; Index]); % new data with resorted vertexes

% rewrite file with  new indexes
fileID = fopen(filepath,'w');
for i=1:numel(DataNew)
    fprintf(fileID,[DataNew{i} '\n']);
end
fclose(fileID);

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);

IndexC = strfind(CStr, 'DataAssociation');
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));

for j=1:length(Index)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',...
        Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    fclose(fileID);
    
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName),'r');
    Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
    CStrGT = Data{1};
    fclose(fileID);
    searchedStr = strcat({'EDGE_2POINTS_SE3Motion'},{' '},{num2str(index1)},...
        {' '},{num2str(index2)},{' '});
    IndexC = strfind(CStrGT, searchedStr);
    edgeIndex = find(~cellfun('isempty', IndexC),1);
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName),'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',edgeIndex-1);
    line = cell2mat(line{1,1});
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
    CStr(Index(j)) = cellstr(sprintf('%s %d %d %d %f %f %f %f %f %f %f %f %f',...
        edgeLabel,index1,index2,index3,edgeValue',edgeCovUT));
%     % Save the file again:
%     fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
%         config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName), 'w');
%     fprintf(fileID, '%s\n', CStr{:});
%     fclose(fileID);
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName(1:end-6),'Test.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end

end