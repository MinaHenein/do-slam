function convertGraphFileToG2O(config,filepath)

%VERTEX_SE3:QUAT i x y z qx qy qz qw 
%VERTEX_TRACKXYZ i x y z

%EDGE_SE3:QUAT i1 i2 x y z qx qy qz qw information
%EDGE_TRACKXYZ i1 i2 x y z information
%upper-triangular block of the information matrix in row-order 

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};

dimPose = config.dimPose+1;
poseVertexFormat  = strcat('%s %d ',repmat(' %f',1,dimPose));
pointVertexFormat = strcat('%s %d ',repmat(' %f',1,config.dimPoint));
poseEdgeFormat    = strcat('%s %d %d ',repmat(' %f',1,dimPose+dimPose*(dimPose+1)/2));
pointEdgeFormat   = strcat('%s %d %d ',repmat(' %f',1,...
    config.dimPoint+config.dimPose*(config.dimPoint+1)/2));

CStrOutput = cell(length(CStr),1);
for i=1:length(CStr)
line = CStr{i,1};
splitLine = strsplit(line,' ');
label = splitLine{1};
switch label
    case config.poseVertexLabel
        g2oLabel = 'VERTEX_SE3:QUAT';
        index = str2double(splitLine{1,2})-1;
        value= [];
        for j=1:config.dimPose
            value = [value,str2double(splitLine{1,3+j-1})];
        end
        wxyz = a2q(value(4:end)');
        xyzw = [wxyz(2),wxyz(3),wxyz(4),wxyz(1)];
        CStrOutput(i) =  cellstr(sprintf(poseVertexFormat,g2oLabel,index,value(1:3),xyzw));
    case config.pointVertexLabel
        g2oLabel = 'VERTEX_TRACKXYZ';
        index = str2double(splitLine{1,2})-1;
        value = [];
        for j=1:config.dimPoint
            value = [value,str2double(splitLine{1,3+j-1})];
        end
        CStrOutput(i) =  cellstr(sprintf(pointVertexFormat,g2oLabel,index,value));
    case config.posePoseEdgeLabel
        g2oLabel = 'EDGE_SE3:QUAT';
        index1 = str2double(splitLine{1,2})-1;
        index2 = str2double(splitLine{1,3})-1;
        value  = [];
        for j=1:config.dimPose
            value = [value,str2double(splitLine{1,4+j-1})];
        end
        covariance = [];
        for j=1:config.dimPose*(config.dimPose+1)/2
            covariance = [ covariance,str2double(splitLine{1,4+config.dimPose+j-1})];
        end
        cov = upperTriVecToCov(covariance);
        poseAxisAngle = [value(4:6) value(1:3)]';
        covPoseAxisAngle = [cov(4:6,4:6) zeros(3);zeros(3) cov(1:3,1:3)];
        [poseQuat, covQuat] = covAxisAngleToCovQuat (poseAxisAngle, covPoseAxisAngle);
        covQuat_t = covQuat(5:7,5:7);
        covQuat_R = covQuat(1:4,1:4);
        infQuat_t = inv(covQuat_t);
        infQuat_R = inv(covQuat_R);
        infQuat = [infQuat_t zeros(3,4); zeros(4,3) infQuat_R];
        CStrOutput(i) =  cellstr(sprintf(poseEdgeFormat,g2oLabel,index1,index2,poseQuat(5:7),...
            poseQuat(1:4),covToUpperTriVec(infQuat)));
    case config.posePointEdgeLabel
        g2oLabel = 'EDGE_TRACKXYZ';
        index1 = str2double(splitLine{1,2})-1;
        index2 = str2double(splitLine{1,3})-1;
        value  = [];
        for j=1:config.dimPoint
            value = [value,str2double(splitLine{1,4+j-1})];
        end
        covarianceVec = [];
        for j=1:config.dimPoint*(config.dimPoint+1)/2
            covarianceVec = [covarianceVec,str2double(splitLine{1,4+config.dimPoint+j-1})];
        end
        CStrOutput(i) =  cellstr(sprintf(pointEdgeFormat,g2oLabel,index1,index2,value,...
            covToUpperTriVec(inv(upperTriVecToCov(covarianceVec)))));
    otherwise
        disp('Unimplemented G2O label type')
        continue
end
end
fclose(fileID);

% Save in a different file
filepath = strcat(filepath(1:end-6),'G2O.graph');
fileID = fopen(filepath,'w');
fprintf(fileID, '%s\n', CStrOutput{:});
fclose(fileID);

end