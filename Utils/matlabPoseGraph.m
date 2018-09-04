function matlabPoseGraph(filepath)

poseGraph = robotics.PoseGraph3D;

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
dimPose = config.dimPose+1;
for i=1:length(CStr)
line = CStr{i,1};
splitLine = strsplit(line,' ');
label = splitLine{1};
switch label
    case config.posePoseEdgeLabel
        value  = [];
        for j=1:config.dimPose
            value = [value,str2double(splitLine{1,4+j-1})];
        end
        wxyz = a2q(value(4:end)');
        xyzw = [wxyz(2),wxyz(3),wxyz(4),wxyz(1)];
        edgeValue = [value,xyzw]; 
    otherwise
        disp('Unimplemented G2O label type')
        continue
end
relPose = edgeValue;
addRelativePose(poseGraph,relPose)

end
fclose(fileID);

[updatedGraph,solutionInfo] = optimizePoseGraph(poseGraph);
figure
title('Updated Pose Graph')
show(updatedGraph,'IDs','off');


end
