%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 20/08/2017
% Contributors:
%--------------------------------------------------------------------------

function [initialCell1, initialCell2] = recreateInitialVertexes(config,measurementsCell,groundTruthCell)
%RECREATEINITIALVERTEXES creates initial for multiple edge types (comparing
%different motion types) and plots them, also tests diagnostics. ONLY
%CODED FOR R3xso(3) - WILL RESULT IN ERROR OTHERWISE.

%% 1. find row types used to compare recreated initials
rowLengths = cellfun('length',groundTruthCell);
vertexRows = (rowLengths==3);
verticesCell = groundTruthCell(vertexRows,:); % auomatically sorts it too

%% 2. identify poses, points, planes, point
initialCell = {groundTruthCell{1}}; % initialise first vertex
for i=1:numel(measurementsCell)
    currentCell = measurementsCell{i};
    switch currentCell{1}
        case config.posePointEdgeLabel
            inputVertex = initialCell{currentCell{3}};
            inputValue = inputVertex{3};
            outputIndex = currentCell{4};
            measValue = currentCell{5};
            outputValue = RelativeToAbsolutePositionR3xso3(inputValue,measValue);
            initialCell{outputIndex,1} = {config.pointVertexLabel, outputIndex, outputValue};
        case config.posePoseEdgeLabel
            inputVertex = initialCell{currentCell{3}};
            inputValue = inputVertex{3};
            outputIndex = currentCell{4};
            measValue = currentCell{5};
            outputValue = RelativeToAbsolutePoseR3xso3(inputValue,measValue);
            initialCell{outputIndex,1} = {config.poseVertexLabel, outputIndex, outputValue};
        case config.pointPointEdgeLabel
            inputVertex = initialCell{currentCell{3}};
            inputValue = inputVertex{3};
            outputIndex = currentCell{4};
            outputValue = inputValue + currentCell{5};
            initialCell{outputIndex,2} = {config.pointVertexLabel, outputIndex, outputValue};
        case config.point3EdgeLabel
            if strcmp(config.motionModel,'constantSpeed')
                % validates norm, prints output
                index1 = currentCell{3}(1);
                index2 = currentCell{3}(2);
                outputIndex= currentCell{3}(3);
                inputVertex1 = initialCell{index1};
                inputVertex2 = initialCell{index2};
                inputVertex3 = initialCell{outputIndex};
                measValue = currentCell{5};
                expectedValue = norm(inputVertex1{3}-inputVertex2{3})-norm(inputVertex2{3}-inputVertex3{3});
                valDifference = measValue-expectedValue;
                if (valDifference<1e-4)
                    fprintf('3-point norm edge between vertexes %d, %d, and %d is below error.\n',index1,index2,outputIndex);
                else
                    fprintf('3-point norm edge error between vertexes %d, %d, and %d is %f.\n',index1,index2,outputIndex,valDifference);
                end
            elseif strcmp(config.motionModel,'constantVelocity')
                % add a new point vertex alongside the previous one based
                % on this measurement
                index1 = currentCell{3}(1);
                index2 = currentCell{3}(2);
                outputIndex = currentCell{3}(3);
                inputVertex1 = initialCell{index1};
                inputVertex2 = initialCell{index2};
                outputValue = 2*inputVertex1{3}-inputVertex2{3};
                initialCell{outputIndex,2} = {config.pointVertexLabel, outputIndex, outputValue};
            end
        case config.pointVelocityEdgeLabel
            index1 = currentCell{3}(1);
            index2 = currentCell{3}(2);
            outputIndex = currentCell{4};
            inputVertex1 = initialCell{index1};
            inputVertex2 = initialCell{index2};
            if strcmp(config.motionModel,'constantSpeed')
                outputValue = norm(inputVertex1{3}-inputVertex2{3});
                initialCell{outputIndex,1} = {config.velocityVertexLabel, outputIndex, outputValue};
            elseif strcmp(config.motionModel,'constantVelocity')
                % recreate point vertex using THIS one, put another point
                % vertex alongside the previous one
                outputValue = (inputVertex1{3}-inputVertex2{3});
                initialCell{outputIndex,1} = {config.velocityVertexLabel, outputIndex, outputValue};
                outputValue1 = initialCell{outputIndex,1}{3};
                meanOutput = mean([outputValue1 outputValue],2);
                initialCell{outputIndex,2} = {config.velocityVertexLabel, outputIndex, meanOutput};
                pointValue = meanOutput + inputVertex1{3};
                initialCell{index2,2} = {config.pointVertexLabel, index2, pointValue};
            end          
        otherwise
            disp('Unidentified edge type.')
    end
end

initialCell1 = initialCell(:,1);
initialCell2 = initialCell1;
if size(initialCell,2)==2
    for i=1:size(initialCell,1) % creates second cell array with different spanning tree for initialisation
        if ~isempty(initialCell{i,2})
            initialCell2{i,1} = initialCell{i,2};
        end
    end
end

initialCell1 = [initialCell1; measurementsCell];
initialCell2 = [initialCell2; measurementsCell];

%% 3. Compare vertexes
poseVertexValues = []; % stores pose vertexes
threshold = config.threshold;

for i=1:size(initialCell,1)
    for j=1:size(initialCell,2)
        if ~isempty(initialCell{i,j})
            label = initialCell{i}{1};
            if strcmp(label,verticesCell{i}{1})
                valError = initialCell{i,j}{3}-verticesCell{i,1}{3};
                if (valError<threshold)
                    fprintf('%s %d done instance %d is within acceptable range of error.\n',label,i,j)
                else
                    fprintf(['%s %d done instance %d has error of [',cell2mat(repmat({' %f'},1,size(valError,1))),'].\n'],label,i,j,valError')
                end
            else
                fprintf('Vertex %d does not match\n',i)
            end
        end
    end
end

%% 4. Plot the initial
poses = [];
points = [];
points2 = [];

extraPoints = (size(initialCell,2)>1);
for i=1:size(initialCell,1)
    switch initialCell{i,1}{1}
        case config.poseVertexLabel
            poses = [poses, initialCell{i,1}{3}];
        case config.pointVertexLabel
            points = [points, initialCell{i,1}{3}];
    end
    if (extraPoints) && ~isempty(initialCell{i,2}) && strcmp(initialCell{i,2}{1},config.pointVertexLabel)
        points2 = [points2, initialCell{i,2}{3}];
    end
end

%% 4.1 Plot poses
figure()
plotGraphFile(config,groundTruthCell,[0 0 1]);
hold on

graphColour = [1 0 0];
plot3(poses(1,:),poses(2,:),poses(3,:),'Color',graphColour,'Marker','o','LineStyle','none');
for i = 1:size(poses,2)
    iPose = poses(:,i);
    if strcmp(config.poseParameterisation,'SE3')
        iPose = LogSE3_Rxt(iPose);
    end
    
    scale = 0.2;
    plotCoordinates(iPose(1:3,:),scale*rot(iPose(4:6,1)))
%     plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
%     scale = 0.5;
%     plotCoordinates(iPose(1:3),scale*rot(iPose(4:6)))
%     plotiCamera.Opacity = 0.1;
%     plotiCamera.Size = 0.1;
%     plotiCamera.Color = graphColour;
end

%% 4.2 Plot points
plotPoints = plot3(points(1,:),points(2,:),points(3,:),'.');
set(plotPoints,'Color',graphColour)
set(plotPoints,'MarkerSize',3)
% set(plotPoints,'MarkerSize',8)
if extraPoints
    plotPoints2 = plot3(points2(1,:),points2(2,:),points2(3,:),'.');
    set(plotPoints2,'Color',[0 1 0]);
    set(plotPoints2,'MarkerSize',3);
end

xlabel('x')
ylabel('y')
zlabel('z')
hold on
axis equal
view([-50,25])

end
