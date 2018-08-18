function produceReducedGraphFile(filepathGT,filepathMeas,nPosesRequired)

%GT
fileID = fopen(filepathGT,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
nPoses = 0;
foundFirstDataAssociation = 0;
posesDone = 0;
x = 0;
for j=1:1:length(CStr)
    splitLine = strsplit(CStr{j,1},' ');
    label = splitLine{1,1};
    if strcmp(label(1:length('VERTEX_POSE')),'VERTEX_POSE')
        nPoses = nPoses+1;
        if nPoses == nPosesRequired
            posesDone = 1;
        end
        if nPoses > nPosesRequired && posesDone
            afterLastPoseIndex = str2double(splitLine{1,2});
            afterLastPoseLine = j;
            posesDone = 0;
             x = 1;
        end
    end
    if ~foundFirstDataAssociation && strcmp(splitLine{1,1},'2POINTS_DataAssociation')
        firstDataAssociationLine = j;
        foundFirstDataAssociation = 1;
    end
    if strcmp(splitLine{1,1},'2POINTS_DataAssociation')
        point2Index = str2double(splitLine{1,3});
        if  x && point2Index < afterLastPoseIndex
            lastDataAssociationLine = j;
        end
    end
end
fclose(fileID);

CStr(lastDataAssociationLine+1:end) = [];
CStr(afterLastPoseLine:firstDataAssociationLine-1) = [];


toWriteGTFilePath = strcat(filepathGT(1:end-6),num2str(nPosesRequired),'.graph');
fileID = fopen(toWriteGTFilePath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);

%Meas
fileID = fopen(toWriteGTFilePath, 'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
IndexC = strfind(CStr, 'VERTEX_POSE');
Index = find(~cellfun('isempty', IndexC));
lastPoseLine = CStr{Index(end),1};
lastPoseLineSplit = strsplit(lastPoseLine,' ');
lastPoseID =  str2double(lastPoseLineSplit{1,2});
IndexC = strfind(CStr, 'VERTEX_POINT');
Index = find(~cellfun('isempty', IndexC));
lastPointLine = CStr{Index(end),1};
lastPointLineSplit = strsplit(lastPointLine,' ');
lastPointID =  str2double(lastPointLineSplit{1,2});
lastDataAssociationLine = CStr{end,1};
IndexC = strfind(CStr, '2POINTS_DataAssociation');
Index = find(~cellfun('isempty', IndexC));
firstDataAssociationLine = CStr{Index(1),1};
fclose(fileID);

fileID = fopen(filepathMeas, 'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};

IndexC = strfind(CStr, strcat({'EDGE_3D'},{' '},{num2str(lastPoseID)},...
    {' '},{num2str(lastPointID)},{' '}));
lastPointMeasLine = find(~cellfun('isempty', IndexC));

IndexC = strfind(CStr, lastDataAssociationLine);
lastDataAssociationLine = find(~cellfun('isempty', IndexC));

IndexC = strfind(CStr, firstDataAssociationLine);
firstDataAssociationLine = find(~cellfun('isempty', IndexC));

CStr(lastDataAssociationLine+1:end) = [];
CStr(lastPointMeasLine+1:firstDataAssociationLine-1) = [];

toWriteMeasFilePath = strcat(filepathMeas(1:end-6),num2str(nPosesRequired),'.graph');
fileID = fopen(toWriteMeasFilePath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);
end