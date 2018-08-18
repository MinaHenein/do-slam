function [nPoses,nPoints,nMotionVertices,nMotionEdges] = countPosesPointsMotion(fileName)

folderPath = pwd;
filepath = strcat(folderPath,'/Data/GraphFiles/',fileName);

% count number of camera poses
nPoses = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX_POSE_R3_SO3')),'VERTEX_POSE_R3_SO3')
        nPoses = nPoses+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
% count number of points
nPoints = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX_POINT_3D')),'VERTEX_POINT_3D')
        nPoints = nPoints+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
% count number of motion vertices
nMotionVertices = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('VERTEX_SE3Motion')),'VERTEX_SE3Motion')
        nMotionVertices = nMotionVertices+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
% count number of motion edges
nMotionEdges = 0;
fileID = fopen(filepath,'r');
line = fgetl(fileID);
while ischar(line)
    if strcmp(line(1:length('2POINTS_DataAssociation')),'2POINTS_DataAssociation')
        nMotionEdges = nMotionEdges+1;
    end
    line = fgetl(fileID);
end
fclose(fileID);
