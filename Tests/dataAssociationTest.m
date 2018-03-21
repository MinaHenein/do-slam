function dataAssociationTest(config,fileName,nObjects)

filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,...
    fileName);

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, 'DataAssociation');
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
objectPts = cell(nObjects,1);
for j=1:1:length(Index)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    pt1 = str2double(splitLine{1,2});
    pt2 = str2double(splitLine{1,3});
    object = str2double(splitLine{1,4});
    objectPts{object} = [objectPts{object} pt1 pt2];
end

for i = 1:size(objectPts,1)
    objectPts{i} = unique(objectPts{i,:});
end

% wrong pts are points that belong to more than 1 object
wrongPts = [];
for i = 1:size(objectPts,1)
    for j = i+1:size(objectPts,1)
      wrongPts = [wrongPts, intersect(objectPts{i,:},objectPts{j,:})];
    end 
end

%find edges with wrong pts
wrongEdges = []; 
for j=1:1:length(Index)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    pt1 = str2double(splitLine{1,2});
    pt2 = str2double(splitLine{1,3});
    if any(wrongPts == pt1) || any(wrongPts == pt2)
        wrongEdges = [wrongEdges, Index(j)];
    end
end
CStr2 = CStr;
CStr2(wrongEdges) = [];

% Save the file again:
fileID = fopen(filepath, 'w');
fprintf(fileID, '%s\n', CStr2{:});
fclose(fileID);

end


