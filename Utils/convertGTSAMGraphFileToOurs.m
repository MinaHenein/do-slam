function convertGTSAMGraphFileToOurs(config,filepath)

%EDGE3 i1 i2 x y z ox oy oz information
%upper-triangular block of the information matrix in row-order 

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};

dimPose = config.dimPose;
poseEdgeFormat    = strcat('%s %d %d ',repmat(' %f',1,dimPose+dimPose*(dimPose+1)/2));

CStrOutput = cell(length(CStr),1);
for i=1:length(CStr)
line = CStr{i,1};
splitLine = strsplit(line,' ');
label = splitLine{1};
switch label
    case 'EDGE3'
        %% To fix quaternion covariance
        ourLabel = 'EDGE_R3_SO3';
        index1 = str2double(splitLine{1,2})+1;
        index2 = str2double(splitLine{1,3})+1;
        value  = [];
        for j=1:config.dimPose
            value = [value,str2double(splitLine{1,4+j-1})];
        end
        axisAngle = value(4:end)';
        covarianceVec = [];
        for j=1:config.dimPose*(config.dimPose+1)/2
            covarianceVec = [covarianceVec,str2double(splitLine{1,4+config.dimPose+j-1})];
        end
        covAxisAngle = upperTriVecToCov(covarianceVec);
        CStrOutput(i) =  cellstr(sprintf(poseEdgeFormat,ourLabel,index1,index2,value(1:3),...
            axisAngle,covToUpperTriVec(inv(covAxisAngle))));
    otherwise
        disp('Unimplemented GTSAM label type')
        continue
end
end
fclose(fileID);

% Save in a different file
filepath = strcat(filepath(1:end-4),'DO-SLAM.graph');
fileID = fopen(filepath,'w');
fprintf(fileID, '%s\n', CStrOutput{:});
fclose(fileID);

end