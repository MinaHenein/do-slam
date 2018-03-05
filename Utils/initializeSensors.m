function  initialSensorPoses = initializeSensors(config,nSensors)
% INITIALIZESENSORS initializes nSensors in a multi-camera system problem

initialSensorPoses = zeros(config.dimPose,nSensors);

GTFileName = config.groundTruthFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,GTFileName);
fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
Index = find(contains(CStr,'VERTEX_POSE_R3_SO3'));
sensorVertexIndexes = zeros(nSensors,1);
for j=1:length(Index)
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',...
        Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index = str2double(splitLine{1,2});
    fclose(fileID);
    sensorVertexIndexes(j,1) = index;
end

MeasurementsFileName = config.measurementsFileName;
filepath = strcat(config.folderPath,config.sep,'Data',...
    config.sep,config.graphFileFolderName,config.sep,MeasurementsFileName);
fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);
Index = find(contains(CStr,'EDGE_2POINTS_SE3Motion'));

pointPairs = zeros(length(Index),2);
for j=1:length(Index)
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',...
        Index(j)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    index1 = str2double(splitLine{1,2});
    index2 = str2double(splitLine{1,3});
    fclose(fileID);
    pointPairs(j,:) = [index1,index2];
end

%% find 3D positions of moving points seen by camera i-1
for i = 2:nSensors
Index = find(contains(CStr,strcat({'EDGE_3D'},{' '},num2str(sensorVertexIndexes(i-1,1)),{' '})));
fromPoints = zeros(3,length(Index));
toPoints = zeros(3,length(Index));
for j = 1:length(Index)
   fromPoints(:,j) = obj.vertices.value(pointPairs(Index(j,1),1));
   toPoints(:,j) = obj.vertices.value(pointPairs(Index(j,1),2));
end

%% estimate their motion --kabsch
[R,t,~] = kabsch(fromPoints,toPoints);

%% use the estimated motion to find their 3D positions at the time they will
%% be seen by camera i
lastSeenPreviouSensor;
firstSeenCurrentSensor;

for j = lastSeenPreviousSensor:firstSeenCurrentSensor
  newPoints(:,j) = [R t; zeros(1,3) 1]*[toPoints(:,j);1];  
end




%% find their measurements in camera i frame


%% triangulate to find an initialization for camera i pose

end
end