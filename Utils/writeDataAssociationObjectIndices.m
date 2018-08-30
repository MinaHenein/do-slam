function writeDataAssociationObjectIndices(config,nObjects)

GTFileName = config.groundTruthFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName);

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
% count number of vertices
nVertices = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
poseVertices = [];
while ischar(line)
    if strcmp(line(1:length('VERTEX')),'VERTEX')
        nVertices = nVertices+1;
    end
    if strcmp(line(1:length(config.poseVertexLabel)),config.poseVertexLabel)
        splitLine = strsplit(line,' ');
        poseVertices = [poseVertices;str2double(splitLine(2))];
    end
    line = fgetl(fileID);
end
fclose(fileID);

nSE3MotionVertices = 0;
isNewSE3Vertex = 0;
object = 0;
lastObject = 0;
index1Last = 0;
index2Last = 0;
nLandmarksPerMotionVertex = 0;
nObjectPosesPerMotionVertex = 0;
for j=1:1:length(Index)
    if j > 1
        lastObject = object;
        index1Last = index1;
        index2Last = index2;
    end
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    object = str2double(splitLine{1,4});
    fclose(fileID);
    if nSE3MotionVertices == 0
        isNewSE3Vertex = 1;
        newVertexID = nVertices +1;
        nLandmarksPerMotionVertex = 2;
        nObjectPosesPerMotionVertex = 1;
    end
    if j>1 && nObjects > 1 && object~=lastObject
        nLandmarksPerMotionVertex = 0;
        nObjectPosesPerMotionVertex = 0;
    end
    
    closestPose1index = sum((index1-poseVertices)>0);
    closestPose2index = sum((index2-poseVertices)>0);
    closestPose1Lastindex = sum((index1Last-poseVertices)>0);
    closestPose2Lastindex = sum((index2Last-poseVertices)>0);
    
    if j > 1
        if (nLandmarksPerMotionVertex < config.newMotionVertexPerNLandmarks) ...
                && object==lastObject
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
        
        if (closestPose1index ~= closestPose1Lastindex || ...
                closestPose2index ~= closestPose2Lastindex) && object==lastObject
            nObjectPosesPerMotionVertex = nObjectPosesPerMotionVertex + 1;
        end
        if nObjectPosesPerMotionVertex >= config.newMotionVertexPerNObjectPoses
            isNewSE3Vertex = 1;
            newVertexID = newVertexID+1;
            nObjectPosesPerMotionVertex = 1;
        end
        
    end
    
    
    if ~isempty(Index(j))
        label = '2PointsDataAssociation';
        index3 = newVertexID;
        if  isNewSE3Vertex
            nSE3MotionVertices = nSE3MotionVertices +1;
        end
        CStr(Index(j))= cellstr(sprintf('%s %d %d %d',...
            label,index1,index2,index3));
        isNewSE3Vertex = 0;
    end
    % Save in the file again
    %     fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
    %         config.sep,config.graphFileFolderName,config.sep,GTFileName), 'w');
    %     fprintf(fileID, '%s\n', CStr{:});
    %     fclose(fileID);
    
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,GTFileName(1:end-6),'Test.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end
newGTFileName = strcat(GTFileName(1:end-6),'Test.graph');
GTFilePath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,newGTFileName);
deleteDataAssociationFromGraphFile(GTFilePath)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MeasurementsFileName = config.measurementsFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName);

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
    searchedStr = strcat({'2PointsDataAssociation'},{' '},{num2str(index1)},...
        {' '},{num2str(index2)},{' '});
    IndexC = strfind(CStrGT, searchedStr);
    edgeIndex = find(~cellfun('isempty', IndexC),1);
    fileID = fopen(strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,newGTFileName),'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',edgeIndex-1);
    line = cell2mat(line{1,1});
    splitLine = str2double(strsplit(line,' '));
    index3 = splitLine(1,4);
    fclose(fileID);
    CStr(Index(j)) = cellstr(sprintf('%s %d %d %d',...
        '2PointsDataAssociation',index1,index2,index3));
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