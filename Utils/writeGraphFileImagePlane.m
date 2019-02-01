function writeGraphFileImagePlane(config)

filepathGT = strcat(config.folderPath,'/Data/GraphFiles/',config.groundTruthFileName);
filepathMeas = strcat(config.folderPath,'/Data/GraphFiles/',config.measurementsFileName);

fileGTID = fopen(filepathGT,'r');
DataGT = textscan(fileGTID,'%s','delimiter','\n','whitespace',' ');
CStrGT = DataGT{1};
fclose(fileGTID);

fileMeasID = fopen(filepathMeas,'r');
DataMeas = textscan(fileMeasID,'%s','delimiter','\n','whitespace',' ');
CStrMeas = DataMeas{1};
fclose(fileMeasID);

IndexC = strfind(CStrMeas, config.posePointEdgeLabel);
Index = find(~cellfun('isempty', IndexC));

pointPixels = [];

for i=1:length(Index)
    line        = strsplit(CStrMeas{Index(i),1},' ');
    poseVertex  = str2double(line{1,2});
    pointVertex = str2double(line{1,3});
    if length(line)== 12 % pixel value not provided
        IndexPose = strfind(CStrGT, strcat({config.poseVertexLabel},{' '},...
            {num2str(poseVertex)},{' '}));
        lineIndexPose = find(~cellfun('isempty', IndexPose));
        linePose = strsplit(CStrGT{lineIndexPose,1},' ');
        poseVertexValue  = str2double(linePose(1,3:end))';
        
        IndexPoint = strfind(CStrGT, strcat({config.pointVertexLabel},{' '},...
            {num2str(pointVertex)},{' '}));
        lineIndexPoint = find(~cellfun('isempty', IndexPoint));
        linePoint = strsplit(CStrGT{lineIndexPoint,1},' ');
        pointVertexValue  = str2double(linePoint(1,3:end))';
        
        pointImagePlane = config.absoluteToRelativePointHandle(poseVertexValue,...
            pointVertexValue,config.intrinsics,config.R);
        
        if isempty(pointPixels) || ~ismember(pointVertex,pointPixels(:,1))
            pointPixels(end+1,:) = [pointVertex,pointImagePlane(1),pointImagePlane(2)];
        end
        
        edgeLabel = 'EDGE_2D_PIXEL';
        edgeValue = pointImagePlane;      
        mu = [0;0];
        sigma = config.stdPosePixel;
        if ~strcmp(config.noiseModel,'Off')
            edgeValue = edgeValue(1:2,1) + normrnd(mu,sigma);
        end
        edge3DValue = [str2double(line{1,4});str2double(line{1,5});str2double(line{1,6})];
        edge3DCovUT = [str2double(line{1,7});str2double(line{1,8});str2double(line{1,9});...
            str2double(line{1,10});str2double(line{1,11});str2double(line{1,12})];
        edgeCovUT = covToUpperTriVec([sigma(1)^2 0; 0 sigma(2)^2]);
        CStrMeas(Index(i)) = cellstr(sprintf('%s %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f',...
            edgeLabel,poseVertex,pointVertex,edge3DValue',edgeValue',edge3DCovUT,edgeCovUT));
    end
end
% Save in a different file
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,...
    config.measurementsFileName(1:end-6),'_Test.graph');
fileID = fopen(filepath,'w');
fprintf(fileID, '%s\n', CStrMeas{:});
fclose(fileID);

IndexC = strfind(CStrGT, config.pointVertexLabel);
Index = find(~cellfun('isempty', IndexC));
for i=1:length(Index)
    line   = strsplit(CStrGT{Index(i),1},' ');
    label  = line{1,1};
    pointVertex = str2double(line{1,2});
    pointVertexValue  = str2double(line(1,3:end))';
    pointImagePlane = pointPixels(pointPixels(:,1)==pointVertex,2:3);
    if length(line)== 5 % pixel value not provided
        CStrGT(Index(i)) = cellstr(sprintf('%s %d %f %f %f %f %f',...
            label,pointVertex,pointVertexValue,pointImagePlane));
    end
end
% Save in a different file
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,...
    config.groundTruthFileName(1:end-6),'_Test.graph');
fileID = fopen(filepath,'w');
fprintf(fileID, '%s\n', CStrGT{:});
fclose(fileID);


end