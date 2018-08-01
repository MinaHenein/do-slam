function corruptDataAssociation(config,percentage)

GTFileName = config.groundTruthFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName);

fileID = fopen(filepath,'r');
line = fgetl(fileID);
objectVertices = [];
while ischar(line)
    if strcmp(line(1:length(config.pointsDataAssociationLabel)),...
            config.pointsDataAssociationLabel)
        splitLine = strsplit(line,' ');
        objectVertices = [objectVertices;str2double(splitLine(4))];
    end
    line = fgetl(fileID);
end
fclose(fileID);

nObjectVertices = length(objectVertices);
corruptedObjectVertices = objectVertices;
for i=1:ceil(percentage*nObjectVertices)
    toCorruptindex = randi([1 nObjectVertices],1);
    toBeCopiedIndex = randi([1 nObjectVertices],1);
    corruptedObjectVertices(toCorruptindex) = objectVertices(toBeCopiedIndex);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MeasurementsFileName = config.measurementsFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName);

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
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    object = corruptedObjectVertices(j);
    fclose(fileID);
    label = '2PointsDataAssociation';
    CStr(Index(j))= cellstr(sprintf('%s %d %d %d',...
        label,index1,index2,object));
end
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName(1:end-6),'Corrupted.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStr{:});
    fclose(fileID);
end
