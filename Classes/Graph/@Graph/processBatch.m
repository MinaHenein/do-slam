function [solver] = processBatch(obj,config,measurementsCell,groundTruthCell)
%PROCESSBATCH Processes all measurements, builds linear system and
%then solves
%   Current implementation still generates initial linearisation points in
%   the same way as incremental - start with initial pose prior, estimate
%   points and next pose from this etc
%   Improved batch solving could be achieved by generating linearisation 
%   points from a different spanning tree

%% 1. Adjust measurementsCell
%convert each element of measurementsCell to a row
measurementsCell = reshapeCell(measurementsCell,'array');

%create prior
%find odometry rows
odometryRows = find(strcmp({measurementsCell{:,1}}',config.posePoseEdgeLabel));
odometryIndex = 1; %first pose
startPoseVertex = measurementsCell{odometryRows(odometryIndex),3};
poseRows = [];
for i = 1:numel(groundTruthCell)
    if strcmp(groundTruthCell{i}{1},config.poseVertexLabel)
        poseRows = [poseRows,i];
    end
end
startPoseValue = groundTruthCell{poseRows(odometryIndex)}{3};
startPoseCovariance = config.covPosePrior;
priorLine = {config.posePriorEdgeLabel,1,[],startPoseVertex,startPoseValue,startPoseCovariance};

%add prior line
measurementsCell = vertcat(priorLine,measurementsCell);

%% 2. Construct vertices and edges at each time step
nVertices = max([measurementsCell{:,4}]);
nEdges = size(measurementsCell,1);

%indexing
iPoseVertices = [];
iPointVertices = [];
iEntityVertices = [];
iObjectVertices = [];

%loop over nSteps
nSteps = numel(odometryRows) + 1;
for i = 1:nSteps
    %identify rows from this time step
    %add elements so formula for iRows works for first and last steps
    odometryRows = [1; find(strcmp(measurementsCell(:,1),config.posePoseEdgeLabel)); size(measurementsCell,1)+1];
    iRows = odometryRows(i):odometryRows(i+1)-1;
    nRows = numel(iRows);
    
    %loop over rows
    for j = 1:nRows
        jRow = measurementsCell(iRows(j),:);
        switch jRow{1}
            case config.posePriorEdgeLabel %posePrior
                %edge index
                jRow{2} = obj.nEdges+1;
                %construct pose vertex
                obj = obj.constructPoseVertex(config,jRow);
                %construct prior edge
                obj = obj.constructPosePriorEdge(config,jRow);
            case config.posePoseEdgeLabel %odometry
                %edge index
                jRow{2} = obj.nEdges+1;
                %construct pose vertex
                obj = obj.constructPoseVertex(config,jRow);
                %construct pose-pose edge
                obj = obj.constructPosePoseEdge(config,jRow);
            case config.posePointEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create point vertex if it doesn't exist
                if jRow{4} > obj.nVertices
                    obj = obj.constructPointVertex(config,jRow);
                end
                %construct pose-point edge
                obj = obj.constructPosePointEdge(config,jRow);
            case config.pointPointEdgeLabel                
                %edgeIndex
                jRow{2} = obj.nEdges+1;
                % construct point-point edge
                obj = obj.constructPointPointEdge(config,jRow);
            case config.point3EdgeLabel
                %edgeIndex
                jRow{2} = obj.nEdges+1;
                % construct 3-points edge 
                if strcmp(config.motionModel,'constantSpeed')
                    obj = obj.construct3PointsEdge(config,jRow);
                elseif strcmp(config.motionModel,'constantVelocity')
                    obj = obj.construct3PointsEdge_v2(config,jRow);
                else
                    error('Motion model not implemented');
                end
            case config.pointVelocityEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create velocity vertex if it doesn't exist
                if jRow{4} > obj.nVertices
                    %find all point vertices connected to this velocity
                    %vertex
                    pointRows = iRows([measurementsCell{iRows,4}]==jRow{4});
                    pointVertices = [measurementsCell{pointRows,3}]';
                    if strcmp(config.motionModel,'constantSpeed')
                        obj = obj.constructVelocityVertex(config,jRow,unique(pointVertices));
                    elseif strcmp(config.motionModel,'constantVelocity')
                        obj = obj.constructVelocityVertex_v2(config,jRow,unique(pointVertices));
                    else
                        error('Motion model not implemented');
                    end
                end
                % construct point-point edge - both points should already exist
                if strcmp(config.motionModel,'constantSpeed')
                    obj = obj.construct2PointsVelocityEdge(config,jRow);
                elseif strcmp(config.motionModel,'constantVelocity')
                    obj = obj.construct2PointsVelocityEdge_v2(config,jRow);
                else
                    error('Motion model not implemented');
                end
            case config.pointPlaneEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create plane vertex if it doesn't exist
                if jRow{4} > obj.nVertices
                    %find all point vertices connected to this plane
                    pointRows = iRows([measurementsCell{iRows,4}]==jRow{4});
                    pointVertices = [measurementsCell{pointRows,3}]';
                    obj = obj.constructPlaneVertex(config,jRow,pointVertices);
                end
                %construct point-plane edge
                obj = obj.constructPointPlaneEdge(config,jRow);
            case config.planePriorEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %construct plane vertex
                %find all point vertices connected to this plane
                pointRows = iRows([measurementsCell{iRows,4}]==jRow{3});
                pointVertices = [measurementsCell{pointRows,3}]';
                %remove plane vertex that doesn't exist
                pointVertices(pointVertices>obj.nVertices) = [];
                %adjust jRow - constructor requires plane as output vertex
                jRow{4} = jRow{3};
                obj = obj.constructPlaneVertex(config,jRow,pointVertices);
                
                %construct plane prior edge
                iPlaneVertex = jRow{3};
                planeNormal = obj.vertices(iPlaneVertex).value(1:3);
                %edge value
                jRow{5} = planeNormal'*planeNormal - 1;
                %edge covariance
                jRow{6} = config.covPlaneNormal;
                obj = obj.constructPlanePriorEdge(config,jRow);    
            case config.angleEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create angle edge if it doesn't exist
                if jRow{4} > obj.nVertices
                    obj = obj.constructAngleVertex(config,jRow);
                end
                obj = obj.constructAngleEdge(config,jRow);
            case config.fixedAngleEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                obj = obj.constructFixedAngleEdge(config,jRow);
            case config.distanceEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                if jRow{4} > obj.nVertices
                    obj = obj.constructDistanceVertex(config,jRow);
                end
                obj = constructDistanceEdge(config,jRow);
            case config.fixedDistanceEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                obj = obj.constructFixedDistanceEdge(config,jRow);
            otherwise; error('%s type invalid',label)
        end
        %construct edge
    end
    %display progress
    if config.displayProgress
        fprintf('\n----------------------------------\n')
        fprintf('Vertices:\t%d/%d\n',obj.nVertices,nVertices)
        fprintf('Edges:\t\t%d/%d\n',obj.nEdges,nEdges)
    end
    
end

%% Solve
%adjust angle constraints
if config.automaticAngleConstraints
    [obj,measurementsCell] = obj.adjustAngleConstraints(measurementsCell);
end

%reorder vertices and edges
measurementsCellCurrent = measurementsCell;
if config.sortVertices
    [obj,newToOldVertices,measurementsCellCurrent] = obj.sortVertices(measurementsCellCurrent);
end
if config.sortEdges
    [obj,newToOldEdges,measurementsCellCurrent] = obj.sortEdges(measurementsCellCurrent);
end

%construct linear system, solve
solver = NonlinearSolver(config);
solver = solver.solve(config,obj,measurementsCellCurrent);

%undo reordering (so indexes will match GT)
if config.sortEdges
    [obj] = obj.unsortEdges(newToOldEdges);
end
if config.sortVertices
    [obj] = obj.unsortVertices(newToOldVertices);
end

end

