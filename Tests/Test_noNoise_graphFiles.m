function Test_noNoise_graphFiles(gtFilepath,measFilepath)

gtFileID = fopen(gtFilepath,'r');
gtData = textscan(gtFileID,'%s','delimiter','\n','whitespace',' ');
gtCStr = gtData{1};
fclose(gtFileID);

measFileID = fopen(measFilepath,'r');
measData = textscan(measFileID,'%s','delimiter','\n','whitespace',' ');
measCStr = measData{1};
fclose(measFileID);
IndexC = strfind(measCStr,'EDGE_3D');
pointMeasIndex = find(~cellfun('isempty', IndexC));
%% 3D landmarks observatins
for i=1:length(pointMeasIndex)
    measFileID = fopen(measFilepath,'r');
    line = textscan(measFileID,'%s',1,'delimiter','\n','headerlines',pointMeasIndex(i)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    cameraIndex = str2double(splitLine{1,2});
    pointIndex = str2double(splitLine{1,3});
    pointMeas = str2double(splitLine{1,4:6});
    fclose(measFileID);
    
    cameraStr = strcat({'VERTEX_POSE_R3_SO3'},{' '},{num2str(cameraIndex)},{' '});
    cameraLineIndex = strfind(gtCStr,cameraStr);
    cameraLine = find(~cellfun('isempty',cameraLineIndex),1);
    gtFileID = fopen(gtFilepath,'r');
    line = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines',cameraLine-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    cameraPose = str2double(splitLine{1,3:end});
    
    pointStr = strcat({'VERTEX_POINT_3D'},{' '},{num2str(pointIndex)},{' '});
    pointLineIndex = strfind(gtCStr,pointStr);
    pointLine = find(~cellfun('isempty',pointLineIndex),1);
    line = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines',pointLine-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    pointPosition = str2double(splitLine{1,3:end});
    fclose(gtFileID);
    pointObs = AbsoluteToRelativePositionR3xso3(cameraPose,pointPosition);
    
    assert(norm(pointMeas-pointObs)<1e-10)   
end

%% odometry
measFileID = fopen(measFilepath,'r');
measData = textscan(measFileID,'%s','delimiter','\n','whitespace',' ');
measCStr = measData{1};
fclose(measFileID);
IndexC = strfind(measCStr,'EDGE_R3_SO3');
odometryMeasIndex = find(~cellfun('isempty', IndexC));

for i=1:length(odometryMeasIndex)
    measFileID = fopen(measFilepath,'r');
    line = textscan(measFileID,'%s',1,'delimiter','\n','headerlines',odometryMeasIndex(i)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    camera1Index = str2double(splitLine{1,2});
    camera2Index = str2double(splitLine{1,3});
    odometryMeas = str2double(splitLine{1,4:9});
    fclose(measFileID);
    
    camera1Str = strcat({'VERTEX_POSE_R3_SO3'},{' '},{num2str(camera1Index)},{' '});
    camera1LineIndex = strfind(gtCStr,camera1Str);
    camera1Line = find(~cellfun('isempty',camera1LineIndex),1);
    gtFileID = fopen(gtFilepath,'r');
    line = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines',camera1Line-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    camera1Pose = str2double(splitLine{1,3:end});
    
    camera2Str = strcat({'VERTEX_POSE_R3_SO3'},{' '},{num2str(camera2Index)},{' '});
    camera2LineIndex = strfind(gtCStr,camera2Str);
    camera2Line = find(~cellfun('isempty',camera2LineIndex),1);
    gtFileID = fopen(gtFilepath,'r');
    line = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines',camera2Line-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    camera2Pose = str2double(splitLine{1,3:end});
    fclose(gtFileID);
    odometry = AbsoluteToRelativePoseR3xso3(camera1Pose,camera2Pose);
    
    assert(norm(odometryMeas-odometry)<1e-10)   
end

end
