function writeGraphFileImagePlane(config)

filepathGT = strcat(config.folderPath,'/Data/GraphFiles/',config.groundTruthFileName);
filepathMeas = strcat(config.folderPath,'/Data/GraphFiles/',config.measurementsFileName);

fileMeasID = fopen(filepathMeas,'r');
DataMeas = textscan(fileMeasID,'%s','delimiter','\n','whitespace',' ');
CStrMeas = DataMeas{1};
fclose(fileMeasID);

IndexC = strfind(CStrMeas, config.posePointEdgeLabel);
Index = find(~cellfun('isempty', IndexC));

for i=1:length(Index)
    line        = strsplit(CStrMeas{Index(i),1},' ');
    poseVertex  = str2double(line{1,2});
    pointVertex = str2double(line{1,3});
    fileGTID = fopen(filepathGT,'r');
    DataGT = textscan(fileGTID,'%s','delimiter','\n','whitespace',' ');
    CStrGT = DataGT{1};
    fclose(fileGTID);
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
    
    edgeLabel = 'EDGE_2D_PIXEL';
    edgeValue = pointImagePlane;
    if ~strcmp(config.noiseModel,'Off')
        mu = [0;0];
        sigma = config.stdPosePixel;
        edgeValue = edgeValue(1:2,1);% + normrnd(mu,sigma);
    end
    edgeCovUT = covToUpperTriVec([sigma(1)^2 0; 0 sigma(2)^2]);
    CStrMeas(Index(i)) = cellstr(sprintf('%s %d %d %d %d %f %f %f',...
        edgeLabel,poseVertex,pointVertex,edgeValue',edgeCovUT));
    % Save in a different file
    filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,...
        config.measurementsFileName(1:end-6),'_Test.graph');
    fileID = fopen(filepath,'w');
    fprintf(fileID, '%s\n', CStrMeas{:});
    fclose(fileID);
end

end