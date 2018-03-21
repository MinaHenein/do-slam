function status = graphFileTest(fileName, nObjects, config)

% status = 0 --> graph file contains errors
% status = 1 --> graph file is clean

if nargin == 3
    filepath = strcat(config.folderPath,config.sep,'Data',config.sep,...
    config.graphFileFolderName,config.sep,fileName);
elseif nargin == 2
    filepath = fileName;
end

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
IndexDA = strfind(CStr, '2POINTS_DataAssociation');
IndexOdo = strfind(CStr, 'EDGE_R3xso3');
IndexPtMeas = strfind(CStr, 'EDGE_3D');
IndexPtPlaneMeas = strfind(CStr, 'EDGE_1D');

IndexDataAssociation = find(~cellfun('isempty', IndexDA)); 
IndexOdometry = find(~cellfun('isempty', IndexOdo));
IndexPointMeas = find(~cellfun('isempty', IndexPtMeas));
IndexPointPlaneMeas = find(~cellfun('isempty', IndexPtPlaneMeas));

poseVertices = [];
pointVertices = [];
planeVertices = [];
objectVertices = [];

for j=1:1:length(IndexDataAssociation)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',IndexDataAssociation(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    pointVertices = [pointVertices, str2double(splitLine{1,2}),str2double(splitLine{1,3})];
    if nObjects > 1
        objectVertices = [objectVertices, str2double(splitLine{1,4})];
    else
        objectVertices = [objectVertices, 0];
    end
end

for j=1:1:length(IndexOdometry)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',IndexOdometry(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    poseVertices = [poseVertices, str2double(splitLine{1,2}),str2double(splitLine{1,3})];
end

for j=1:1:length(IndexPointMeas)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',IndexPointMeas(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    poseVertices = [poseVertices, str2double(splitLine{1,2})];
    pointVertices = [pointVertices, str2double(splitLine{1,3})];
end

for j=1:1:length(IndexPointPlaneMeas)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',IndexPointPlaneMeas(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    pointVertices = [pointVertices, str2double(splitLine{1,2})];
    planeVertices = [planeVertices, str2double(splitLine{1,3})];
end

poseVertices = unique(poseVertices);
pointVertices = unique(pointVertices);
planeVertices = unique(planeVertices);
objectVertices = unique(objectVertices);

% wrong vertices are vertices that belong to more than 1 object
wrongVertices = [];
wrongVertices = [wrongVertices, intersect(poseVertices,pointVertices)];
wrongVertices = [wrongVertices, intersect(poseVertices,planeVertices)];
wrongVertices = [wrongVertices, intersect(pointVertices,planeVertices)];

wrongVertices = [wrongVertices, intersect(objectVertices,poseVertices)];
wrongVertices = [wrongVertices, intersect(objectVertices,pointVertices)];
wrongVertices = [wrongVertices, intersect(objectVertices,planeVertices)];

if isempty(wrongVertices)
    status = 1;
else
    status = 0;
end

% if status == 0 && writeNewGraphFile
%     %find edges with wrong vertices
%     wrongEdges = [];
%     for j=1:1:size(CStr,1)
%         % get line of Index
%         fileID = fopen(filepath,'r');
%         line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',j-1);
%         splitLine = strsplit(cell2mat(line{1,1}),' ');
%         vertex1 = str2double(splitLine{1,2});
%         vertex2 = str2double(splitLine{1,3});
%         if strcmp(splitLine{1},'EDGE_1D')
%             vertex3 = str2double(splitLine{1,4});
%         end
%         if any(wrongVertices == vertex1) || any(wrongVertices == vertex2) ...
%                 ||  any(wrongVertices == vertex3)
%             wrongEdges = [wrongEdges, j];
%         end
%     end
%     CStr2 = CStr;
%     CStr2(wrongEdges) = [];
%     % Save the file again:
%     fileID = fopen(filepath, 'w');
%     fprintf(fileID, '%s\n', CStr2{:});
%     fclose(fileID);
% end

end