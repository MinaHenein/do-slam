function writeDataAssociationVerticesEdges_constantSE3MotionNoGT(config)

MeasurementsFileName = config.measurementsFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName);

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
nVertices = 0;
for i = 1:length(Data{1})
    splitLine = strsplit(cell2mat(CStr(i,1)),' ');
    vertex = str2double(splitLine{1,3});
    if vertex > nVertices
        nVertices = vertex;
    end
end
DataAssociation = CStr(contains(CStr,'DataAssociation'));
nObjects = 1;
for i = 1:length(DataAssociation)
    splitLine = strsplit(cell2mat(DataAssociation(i,1)),' ');
    if length(splitLine) > 3
        object = str2double(splitLine{1,4});
        if object > nObjects
            nObjects = object;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% re-order data association entries by object
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
Index = find(contains(CStr,'DataAssociation'));
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
Index = find(contains(CStr,'DataAssociation'));
nSE3MotionVertices = 0;
object = 0;
lastObject = 0;
index2Last = 0;
nLandmarksPerMotionVertex = 0;

for j=1:length(Index)
    if j > 1
        lastObject = object;
        index2Last = index2;
    end   
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',...
        Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    object = str2double(splitLine{1,4});
    
    if nSE3MotionVertices == 0
        newVertexID = nVertices +1;
        nLandmarksPerMotionVertex = 2;
    end
    if nObjects > 1 && object~=lastObject
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
        newVertexID = newVertexID+1;
        nLandmarksPerMotionVertex = 2;
        nSE3MotionVertices = nSE3MotionVertices +1;
        end
    end
    
    sigmaEdge = config.std2PointsSE3Motion;
    covEdge = config.cov2PointsSE3Motion;
    edgeCovUT = covToUpperTriVec(covEdge);
    edgeLabel = config.pointSE3MotionEdgeLabel;
    edgeValue = sigmaEdge.*rand;
    CStr(Index(j)) = cellstr(sprintf('%s %d %d %d %f %f %f %f %f %f %f %f %f',...
        edgeLabel,index1,index2,newVertexID,edgeValue',edgeCovUT));
     % Save the file again:
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName), 'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end

end