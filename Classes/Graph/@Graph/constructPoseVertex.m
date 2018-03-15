function [obj] = constructPoseVertex(obj,config,edgeRow)
%CONSTRUCTPOSEVERTEX Constructs vertex representing a pose. Pose is
%estimated from previous pose and odometry measurement

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
inputVertices = edgeRow{3};
outputVertices = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. compute pose
switch edgeLabel
    case config.posePriorEdgeLabel %prior
        pose2 = edgeValue;
    case config.posePoseEdgeLabel %odometry
        pose1 = obj.vertices(inputVertices).value;
        controlInput = edgeValue;
        switch config.cameraControlInput
            case 'relativePose'
                pose2 = config.relativeToAbsolutePoseHandle(pose1,controlInput);
                %                 pose2(4:6) = wrapAxisAngle(pose2(4:6),'pi');
                %                 pose2 = RelativeToAbsolutePose(pose1,controlInput);
                %                 pose2 = Relative2AbsoluteSE3(pose1,controlInput);
            case 'relativeVelocity'
                pose2 = config.relativeToAbsolutePoseHandle(pose1,controlInput*config.dt);
                %                 pose2(4:6) = wrapAxisAngle(pose2(4:6),'pi');
                %                 pose2 = RelativeToAbsolutePose(pose1,controlInput*config.dt);
                %                 pose2 = Relative2AbsoluteSE3(pose1,controlInput*config.dt);
        end
    case config.posePointEdgeLabel %to initialise a pose from point measurement - multicamera system
        GTFileName = config.groundTruthFileName;
        filepath = strcat(config.folderPath,config.sep,'Data',...
            config.sep,config.graphFileFolderName,config.sep,GTFileName);
        fileID = fopen(filepath,'r');
        Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
        CStr = Data{1};
        fclose(fileID);
        IndexC = strfind(CStr, config.poseVertexLabel);
        % find lines with a DataAssociation entry
        Index = find(~cellfun('isempty', IndexC));
        poseIndexes = [];
        poseValues = [];
        for j=1:length(Index)
            % get line of Index
            fileID = fopen(filepath,'r');
            line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
            line = cell2mat(line{1,1});
            splitLine = strsplit(line,' ');
            index = str2double(splitLine(1,2));
            value = str2double(splitLine(1,3:8));
            poseIndexes = [poseIndexes;index];
            poseValues = [poseValues; value];
            fclose(fileID);
        end
        if ~isempty(find(poseIndexes == inputVertices,1))
            pose2 = poseValues(poseIndexes == inputVertices,:)';
            if ~strcmp(config.noiseModel,'Off')
                for i = 1:size(pose2,2)
                    pose2(:,i) = config.relativeToAbsolutePoseHandle(pose2(:,i),config.stdPosePose(:,i));
                end
            end
            disp(strcat({'camera'},{' '},num2str(find(poseIndexes == inputVertices)),...
                {' '},' initialized'))
        else
            pose2 =  [0;0;0;0;0;0];
        end
        outputVertices = inputVertices;
    otherwise; error('error, wrong edgeLabel')
end

%% 3. vertex properties
value = pose2;
covariance = []; %not using this property yet
type = 'pose';
iEdges = [edgeIndex];
index = outputVertices;

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);

end

