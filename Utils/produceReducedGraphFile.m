function produceReducedGraphFile(filepath, nPosesRequired)

fileID = fopen(filepath,'r');
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

fileID = fopen(filepath, 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);


end