function produceReducedGraphFile(config,filepathGT,filepathMeas,nPosesRequired)

%GT
fileID = fopen(filepathGT,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
nPoses = 0;
for j=1:1:length(CStr)
    splitLine = strsplit(CStr{j,1},' ');
    label = splitLine{1,1}; 
    if strcmp(label(1:length('VERTEX_POSE')),'VERTEX_POSE')
        nPoses = nPoses+1;
        if nPoses == nPosesRequired+1 
            afterLastPoseLine = j;
        end
    end
end
fclose(fileID);

CStr(afterLastPoseLine:end) = [];


toWriteGTFilePath = strcat(filepathGT(1:end-6),num2str(nPosesRequired),'.graph');
fileID = fopen(toWriteGTFilePath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);

%Meas
%determine last line in GT
fileID = fopen(toWriteGTFilePath, 'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
splitLine = strsplit(CStr{end,1},' ');
label = splitLine{1,1};
if strcmp(label(1:length('VERTEX_POSE')),'VERTEX_POSE')
    lastGTLine =  'pose';
elseif strcmp(label(1:length('VERTEX_POINT')),'VERTEX_POINT')
    lastGTLine = 'point';
elseif strcmp(label(1:length('2POINTS_DataAssociation')),'2POINTS_DataAssociation')
    lastGTLine = 'dataAssociation';
end

IndexC = strfind(CStr,'VERTEX_POSE');
Index = find(~cellfun('isempty', IndexC));
lastPoseLine = CStr{Index(end),1};
lastPoseLineSplit = strsplit(lastPoseLine,' ');
lastPoseID =  str2double(lastPoseLineSplit{1,2});
beforeLastPoseLine = CStr{Index(end-1),1};
beforeLastPoseLineSplit = strsplit(beforeLastPoseLine,' ');
beforeLastPoseID =  str2double(beforeLastPoseLineSplit{1,2});

IndexC = strfind(CStr,'VERTEX_POINT');
Index = find(~cellfun('isempty', IndexC));
lastPointLine = CStr{Index(end),1};
lastPointLineSplit = strsplit(lastPointLine,' ');
lastPointID =  str2double(lastPointLineSplit{1,2});

IndexC = strfind(CStr,'2POINTS_DataAssociation');
dataAssociationIndex = find(~cellfun('isempty', IndexC));
fclose(fileID);

fileID = fopen(filepathMeas, 'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};

switch lastGTLine
    case 'pose'
        IndexC = strfind(CStr,strcat({config.posePoseEdgeLabel},{' '},...
            num2str(beforeLastPoseID),{' '},num2str(lastPoseID)));
        Index = find(~cellfun('isempty', IndexC));
        lastLine = Index(end);
    case 'point'
        IndexC = strfind(CStr,strcat({config.posePointEdgeLabel},{' '},...
            num2str(lastPoseID),{' '},num2str(lastPointID)));
        Index = find(~cellfun('isempty', IndexC));
        lastLine = Index(end);
    case 'dataAssociation'
        lastLine = dataAssociationIndex(end);
end

CStr(lastLine+1:end) = [];

toWriteMeasFilePath = strcat(filepathMeas(1:end-6),num2str(nPosesRequired),'.graph');
fileID = fopen(toWriteMeasFilePath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);
end